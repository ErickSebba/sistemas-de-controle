"""
============================================================
 monitor_tempo_real.py
 PENDULO DE FURUTA - Monitor em tempo real com controle interativo
 Entrega 2 - Introducao a Sistemas de Controle - UFMT
============================================================

O QUE FAZ:
  - Conecta no Arduino (sketch furuta_interativo.ino)
  - Plota AO VIVO o PWM aplicado e o angulo do pendulo
  - Controle manual: degrau +, degrau -, freio, zerar encoder
  - TESTE 1 (Varredura): pulsos +/- curtos, varrendo niveis de PWM
  - TESTE 2 (Aleatorio): steps de magnitude e duracao aleatorias,
                         alternando lado, com descanso proporcional ao "chute"
  - Grava CSV (compativel com estimar_tf.py) e estima G(s) na hora

COMO USAR:
  1. Carregue furuta_interativo.ino no Arduino Mega.
  2. Feche o Monitor Serial da Arduino IDE.
  3. python monitor_tempo_real.py   (ou --port COM3)

DEPENDENCIAS:
  pip install pyserial matplotlib numpy

PAINEL DE CONTROLE:
  Manual:  Degrau (PWM) + botoes Degrau+/Degrau-/Freio/Zerar
  Arquivo: Salvar CSV / Estimar G(s) / Limpar tela

  TESTE 1 - VARREDURA (deterministico):
    PWM ini / passo / PWM fim -> niveis percorridos
    pulso (ms)                -> duracao do pulso no PWM inicial
                                 (encolhe sozinho conforme o PWM cresce)
    Botao "Teste varredura"

  TESTE 2 - ALEATORIO:
    PWM ini / PWM fim    -> faixa da magnitude sorteada
    imp min / imp max(ms)-> faixa da duracao do impulso sorteada
    n testes             -> quantos pulsos rodar
    rep min / rep max(s) -> limites do descanso (cresce com o chute)
    Lado alterna sempre: DIR, ESQ, DIR, ESQ...
    Botao "Teste aleatorio"

  rep min / rep max sao usados pelos dois testes.
  Botao "Parar teste" interrompe qualquer teste.
============================================================
"""

import argparse
import csv
import sys
import time
import random
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import numpy as np


BAUD_RATE = 115200
JANELA_S  = 12.0
MAX_PTS   = 5000


# -- Deteccao de porta -----------------------------------------

def detectar_porta(preferida=None):
    portas = list(serial.tools.list_ports.comports())
    if preferida:
        for p in portas:
            if p.device.lower() == preferida.lower():
                return p.device
        print(f"[AVISO] Porta {preferida} nao encontrada.")
    candidatas = [p for p in portas
                  if any(k in (p.description or '').lower()
                         for k in ('arduino', 'ch340', 'usb'))]
    if candidatas:
        return candidatas[0].device
    if portas:
        print("Portas disponiveis:")
        for i, p in enumerate(portas):
            print(f"  [{i}] {p.device} - {p.description}")
        escolha = input("Numero ou nome da porta: ").strip()
        if escolha.isdigit() and int(escolha) < len(portas):
            return portas[int(escolha)].device
        return escolha
    raise RuntimeError("Nenhuma porta serial encontrada.")


# -- Leitor serial em thread separada --------------------------

class LeitorSerial(threading.Thread):
    """Le o Arduino continuamente e armazena os dados num buffer."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.rodando = True
        self.t = deque(maxlen=MAX_PTS)
        self.pwm = deque(maxlen=MAX_PTS)
        self.alpha = deque(maxlen=MAX_PTS)
        self.todos = []          # (tempo_ms, pwm, alpha_rad)
        self.t0_ms = None
        self.lock = threading.Lock()

    def run(self):
        while self.rodando:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            linha = raw.decode('utf-8', errors='ignore').strip()
            if not linha or linha.startswith('#') or linha.startswith('tempo'):
                if linha.startswith('#'):
                    print(f"[Arduino] {linha}")
                continue
            partes = linha.split(',')
            if len(partes) != 3:
                continue
            try:
                tempo_ms = float(partes[0])
                pwm_val  = float(partes[1])
                alpha    = float(partes[2])
            except ValueError:
                continue
            if self.t0_ms is None:
                self.t0_ms = tempo_ms
            t_rel = (tempo_ms - self.t0_ms) / 1000.0
            with self.lock:
                self.t.append(t_rel)
                self.pwm.append(pwm_val)
                self.alpha.append(np.degrees(alpha))
                self.todos.append((int(tempo_ms), int(pwm_val), alpha))

    def parar(self):
        self.rodando = False


# -- Aplicacao principal ---------------------------------------

class MonitorApp:
    def __init__(self, ser):
        self.ser = ser
        self.ser_lock = threading.Lock()
        self.leitor = LeitorSerial(ser)
        self.leitor.start()
        self.degrau = 150

        # -- Parametros dos testes --
        self.pwm_ini   = 100      # PWM ini (varredura) / min da faixa (aleatorio)
        self.pwm_passo = 50       # passo (so varredura)
        self.pwm_fim   = 210      # PWM fim (varredura) / max da faixa (aleatorio)
        self.rep_min   = 5.0      # descanso minimo (s) - dois testes
        self.rep_max   = 8.0      # descanso maximo (s) - dois testes
        self.t_pulso_base_ms = 1.5   # pulso no PWM inicial (varredura)
        self.t_pulso_min_ms  = 0.5
        self.imp_min_ms = 500     # impulso min (aleatorio)
        self.imp_max_ms = 700     # impulso max (aleatorio)
        self.n_testes   = 20      # quantidade de pulsos (aleatorio)
        self.pwm_limite = 255     # teto fisico do analogWrite

        self._teste_rodando = False
        self._teste_abortar = False
        self._teste_info    = ""

        # -- Layout --
        self.fig, (self.ax_u, self.ax_y) = plt.subplots(
            2, 1, figsize=(11, 9.6), sharex=True)
        self.fig.subplots_adjust(bottom=0.50, hspace=0.25)
        self.fig.suptitle("Pendulo de Furuta - Monitor em Tempo Real",
                          fontsize=13, fontweight='bold')

        self.linha_u, = self.ax_u.step([], [], where='post',
                                        color='royalblue', lw=1.8)
        self.ax_u.set_ylabel("PWM"); self.ax_u.grid(alpha=0.3)
        self.ax_u.set_ylim(-270, 270)
        self.ax_u.set_title("Entrada U(t) - PWM aplicado")

        self.linha_y, = self.ax_y.plot([], [], color='darkorange', lw=1.5)
        self.ax_y.set_ylabel("alpha (graus)"); self.ax_y.set_xlabel("Tempo (s)")
        self.ax_y.grid(alpha=0.3)
        self.ax_y.set_title("Saida Y(t) - angulo do pendulo")

        self._criar_controles()

        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._atualizar_plot)
        self.timer.start()

        self.fig.canvas.mpl_connect('close_event', self._on_close)

    # -- Controles --------------------------------------------
    def _criar_controles(self):
        # Linha manual
        ax_box = self.fig.add_axes([0.15, 0.44, 0.06, 0.04])
        self.box_degrau = TextBox(ax_box, "Degrau  ", initial=str(self.degrau))
        self.box_degrau.on_submit(self._set_degrau)

        ax_mais  = self.fig.add_axes([0.30, 0.44, 0.075, 0.04])
        ax_menos = self.fig.add_axes([0.385, 0.44, 0.075, 0.04])
        ax_freio = self.fig.add_axes([0.47, 0.44, 0.075, 0.04])
        ax_zero  = self.fig.add_axes([0.555, 0.44, 0.075, 0.04])
        self.btn_mais  = Button(ax_mais,  "Degrau +",  color='#c8e6c9')
        self.btn_menos = Button(ax_menos, "Degrau -",  color='#ffcdd2')
        self.btn_freio = Button(ax_freio, "Freio (0)", color='#fff9c4')
        self.btn_zero  = Button(ax_zero,  "Zerar a",   color='#bbdefb')
        self.btn_mais.on_clicked(lambda e: self._enviar(f"P{self.degrau}"))
        self.btn_menos.on_clicked(lambda e: self._enviar(f"P{-self.degrau}"))
        self.btn_freio.on_clicked(lambda e: self._enviar("B"))
        self.btn_zero.on_clicked(lambda e: self._enviar("Z"))

        # Linha arquivo
        ax_salvar = self.fig.add_axes([0.15, 0.385, 0.10, 0.04])
        ax_estim  = self.fig.add_axes([0.26, 0.385, 0.10, 0.04])
        ax_limpar = self.fig.add_axes([0.37, 0.385, 0.10, 0.04])
        self.btn_salvar = Button(ax_salvar, "Salvar CSV",   color='#d1c4e9')
        self.btn_estim  = Button(ax_estim,  "Estimar G(s)", color='#b2dfdb')
        self.btn_limpar = Button(ax_limpar, "Limpar tela",  color='#eeeeee')
        self.btn_salvar.on_clicked(self._salvar_csv)
        self.btn_estim.on_clicked(self._estimar)
        self.btn_limpar.on_clicked(self._limpar)

        self.fig.text(0.15, 0.345, "TESTE AUTOMATIZADO",
                      fontsize=9, fontweight='bold', color='#444')

        # PWM (compartilhado)
        ax_pini   = self.fig.add_axes([0.15, 0.285, 0.05, 0.04])
        ax_ppasso = self.fig.add_axes([0.33, 0.285, 0.05, 0.04])
        ax_pfim   = self.fig.add_axes([0.50, 0.285, 0.05, 0.04])
        self.box_pwm_ini   = TextBox(ax_pini,   "PWM ini  ", initial=str(self.pwm_ini))
        self.box_pwm_passo = TextBox(ax_ppasso, "passo  ",   initial=str(self.pwm_passo))
        self.box_pwm_fim   = TextBox(ax_pfim,   "PWM fim  ", initial=str(self.pwm_fim))
        self.box_pwm_ini.on_submit(  lambda t: self._set_param('pwm_ini',   t, int))
        self.box_pwm_passo.on_submit(lambda t: self._set_param('pwm_passo', t, int))
        self.box_pwm_fim.on_submit(  lambda t: self._set_param('pwm_fim',   t, int))

        # Descanso (compartilhado)
        ax_rmin = self.fig.add_axes([0.15, 0.225, 0.05, 0.04])
        ax_rmax = self.fig.add_axes([0.33, 0.225, 0.05, 0.04])
        self.box_rep_min = TextBox(ax_rmin, "rep min (s)  ", initial=str(self.rep_min))
        self.box_rep_max = TextBox(ax_rmax, "rep max (s)  ", initial=str(self.rep_max))
        self.box_rep_min.on_submit(lambda t: self._set_param('rep_min', t, float))
        self.box_rep_max.on_submit(lambda t: self._set_param('rep_max', t, float))

        # Pulso (varredura)
        ax_pulso = self.fig.add_axes([0.15, 0.150, 0.05, 0.04])
        self.box_pulso = TextBox(ax_pulso, "pulso (ms)  ", initial=str(self.t_pulso_base_ms))
        self.box_pulso.on_submit(lambda t: self._set_param('t_pulso_base_ms', t, float))
        self.fig.text(0.225, 0.158, "(Teste varredura)", fontsize=8, color='#888')

        # Impulso + n (aleatorio)
        ax_imin = self.fig.add_axes([0.15, 0.085, 0.05, 0.04])
        ax_imax = self.fig.add_axes([0.33, 0.085, 0.05, 0.04])
        ax_nt   = self.fig.add_axes([0.50, 0.085, 0.05, 0.04])
        self.box_imp_min = TextBox(ax_imin, "imp min (ms)  ", initial=str(self.imp_min_ms))
        self.box_imp_max = TextBox(ax_imax, "imp max (ms)  ", initial=str(self.imp_max_ms))
        self.box_n       = TextBox(ax_nt,   "n testes  ",     initial=str(self.n_testes))
        self.box_imp_min.on_submit(lambda t: self._set_param('imp_min_ms', t, float))
        self.box_imp_max.on_submit(lambda t: self._set_param('imp_max_ms', t, float))
        self.box_n.on_submit(      lambda t: self._set_param('n_testes',   t, int))

        # Botoes de teste
        ax_tvar = self.fig.add_axes([0.62, 0.225, 0.15, 0.05])
        ax_tale = self.fig.add_axes([0.62, 0.150, 0.15, 0.05])
        ax_tsto = self.fig.add_axes([0.62, 0.075, 0.15, 0.05])
        self.btn_varredura = Button(ax_tvar, "Teste varredura", color='#a5d6a7')
        self.btn_aleatorio = Button(ax_tale, "Teste aleatorio", color='#90caf9')
        self.btn_parar     = Button(ax_tsto, "Parar teste",     color='#ef9a9a')
        self.btn_varredura.on_clicked(self._iniciar_teste)
        self.btn_aleatorio.on_clicked(self._iniciar_teste_aleatorio)
        self.btn_parar.on_clicked(self._parar_teste)

        self.txt_status = self.fig.text(0.80, 0.40, "", fontsize=9,
                                        family='monospace', color='navy')

    # -- Setters ----------------------------------------------
    def _set_degrau(self, txt):
        try:
            self.degrau = int(float(txt))
            print(f"[INFO] Degrau = {self.degrau}")
        except ValueError:
            print("[AVISO] Valor invalido para degrau.")

    def _set_param(self, attr, txt, tipo):
        try:
            setattr(self, attr, tipo(float(txt)))
            print(f"[INFO] {attr} = {getattr(self, attr)}")
        except ValueError:
            print(f"[AVISO] Valor invalido para {attr}.")

    def _enviar(self, comando):
        try:
            with self.ser_lock:
                self.ser.write((comando + "\n").encode())
                self.ser.flush()
            print(f"[ENVIADO] {comando}")
        except Exception as e:
            print(f"[ERRO] Falha ao enviar comando: {e}")

    def _limpar(self, event=None):
        with self.leitor.lock:
            self.leitor.t.clear()
            self.leitor.pwm.clear()
            self.leitor.alpha.clear()
            self.leitor.t0_ms = None

    def _espera_interrompivel(self, dur):
        """Espera 'dur' s, podendo ser cortada pelo botao Parar."""
        fim = time.time() + dur
        while time.time() < fim:
            if self._teste_abortar:
                return
            time.sleep(0.02)

    # -- TESTE 1: varredura -----------------------------------
    def _tempo_pulso_s(self, pwm):
        """Pulso encolhe com o PWM p/ manter o chute ~constante
        (deslocamento do braco ~ impulso ~ PWM*tempo): nao enrosca o fio.
            tempo = tempo_base * (pwm_ini / pwm)   (com piso de seguranca)"""
        pwm = max(1, abs(int(pwm)))
        t_ms = self.t_pulso_base_ms * (self.pwm_ini / pwm)
        t_ms = max(self.t_pulso_min_ms, t_ms)
        return t_ms / 1000.0

    def _iniciar_teste(self, event=None):
        if self._teste_rodando:
            print("[AVISO] Teste ja em andamento."); return
        if self.pwm_passo <= 0:
            print("[AVISO] 'passo' deve ser > 0."); return
        if self.pwm_fim < self.pwm_ini:
            print("[AVISO] 'PWM fim' deve ser >= 'PWM ini'."); return
        self._teste_abortar = False
        self._teste_rodando = True
        threading.Thread(target=self._loop_teste, daemon=True).start()
        print("[INFO] Teste varredura iniciado.")

    def _loop_teste(self):
        try:
            pwm = self.pwm_ini
            while pwm <= self.pwm_fim and not self._teste_abortar:
                for sinal, nome in ((+1, "DIR"), (-1, "ESQ")):
                    if self._teste_abortar:
                        break
                    valor   = sinal * pwm
                    t_pulso = self._tempo_pulso_s(pwm)
                    self._teste_info = (f"Varredura: PWM {valor:+d} ({nome})\n"
                                        f"pulso {t_pulso*1000:.1f} ms")
                    print(f"[TESTE] PWM={valor:+d}  pulso={t_pulso*1000:.2f} ms")
                    self._enviar(f"P{valor}")
                    time.sleep(t_pulso)
                    self._enviar("B")
                    if self._teste_abortar:
                        break
                    self._espera_interrompivel(random.uniform(self.rep_min, self.rep_max))
                pwm += self.pwm_passo
        finally:
            self._finalizar_teste()

    # -- TESTE 2: aleatorio -----------------------------------
    def _descanso_por_chute(self, mag, t_imp_s):
        """Descanso cresce com o tamanho do chute (impulso = mag * t_imp).
        Normaliza entre o menor e o maior chute possiveis da faixa
        configurada e mapeia linearmente para [rep_min, rep_max].
        -> chutes pequenos ~ rep_min ; chutes grandes ~ rep_max."""
        kick  = mag * t_imp_s
        pmin, pmax = min(self.pwm_ini, self.pwm_fim), max(self.pwm_ini, self.pwm_fim)
        kmin = pmin * (self.imp_min_ms / 1000.0)
        kmax = pmax * (self.imp_max_ms / 1000.0)
        frac = (kick - kmin) / (kmax - kmin) if kmax > kmin else 0.0
        frac = min(1.0, max(0.0, frac))
        return self.rep_min + frac * (self.rep_max - self.rep_min)

    def _iniciar_teste_aleatorio(self, event=None):
        if self._teste_rodando:
            print("[AVISO] Teste ja em andamento."); return
        if self.pwm_fim < self.pwm_ini:
            print("[AVISO] 'PWM fim' deve ser >= 'PWM ini'."); return
        if self.imp_max_ms < self.imp_min_ms:
            print("[AVISO] 'imp max' deve ser >= 'imp min'."); return
        if self.n_testes <= 0:
            print("[AVISO] 'n testes' deve ser > 0."); return
        self._teste_abortar = False
        self._teste_rodando = True
        threading.Thread(target=self._loop_teste_aleatorio, daemon=True).start()
        print("[INFO] Teste aleatorio iniciado.")

    def _loop_teste_aleatorio(self):
        pmin, pmax = min(self.pwm_ini, self.pwm_fim), max(self.pwm_ini, self.pwm_fim)
        sinal = +1   # comeca pela direita; alterna a cada pulso
        try:
            for i in range(int(self.n_testes)):
                if self._teste_abortar:
                    break
                mag     = random.randint(pmin, pmax)
                mag     = min(mag, self.pwm_limite)
                t_imp_s = random.uniform(self.imp_min_ms, self.imp_max_ms) / 1000.0
                valor   = sinal * mag
                nome    = "DIR" if sinal > 0 else "ESQ"
                descanso = self._descanso_por_chute(mag, t_imp_s)

                self._teste_info = (f"Aleatorio {i+1}/{int(self.n_testes)} ({nome})\n"
                                    f"PWM {valor:+d}  imp {t_imp_s*1000:.0f} ms\n"
                                    f"descanso {descanso:.1f} s")
                print(f"[TESTE] {i+1}/{int(self.n_testes)}  PWM={valor:+d}  "
                      f"imp={t_imp_s*1000:.0f} ms  descanso={descanso:.1f} s")

                self._enviar(f"P{valor}")
                self._espera_interrompivel(t_imp_s)   # impulso (interrompivel)
                self._enviar("B")                      # solta -> resposta livre
                if self._teste_abortar:
                    break
                self._espera_interrompivel(descanso)
                sinal = -sinal                         # troca o lado
        finally:
            self._finalizar_teste()

    def _parar_teste(self, event=None):
        if self._teste_rodando:
            self._teste_abortar = True
            self._enviar("B")
            print("[INFO] Parando teste (freio aplicado).")

    def _finalizar_teste(self):
        self._enviar("B")
        self._teste_rodando = False
        self._teste_info = ("Teste abortado." if self._teste_abortar
                            else "Teste finalizado.")
        print("[INFO] " + self._teste_info)

    # -- Atualizacao do grafico -------------------------------
    def _atualizar_plot(self):
        with self.leitor.lock:
            t = np.array(self.leitor.t)
            u = np.array(self.leitor.pwm)
            y = np.array(self.leitor.alpha)
        if len(t) < 2:
            return

        self.linha_u.set_data(t, u)
        self.linha_y.set_data(t, y)

        t_max = t[-1]
        t_min = max(0, t_max - JANELA_S)
        self.ax_u.set_xlim(t_min, t_max + 0.5)
        self.ax_y.set_xlim(t_min, t_max + 0.5)

        vis = (t >= t_min)
        if vis.any():
            ymin, ymax = y[vis].min(), y[vis].max()
            margem = max(5, 0.1 * (ymax - ymin))
            self.ax_y.set_ylim(ymin - margem, ymax + margem)

        status = (f"PWM atual: {int(u[-1]):+4d}\n"
                  f"alpha: {y[-1]:+6.2f} graus\n"
                  f"amostras: {len(self.leitor.todos)}")
        if self._teste_info:
            status += f"\n{self._teste_info}"
        self.txt_status.set_text(status)

        self.fig.canvas.draw_idle()

    # -- Salvar CSV -------------------------------------------
    def _salvar_csv(self, event=None):
        with self.leitor.lock:
            dados = list(self.leitor.todos)
        if not dados:
            print("[AVISO] Sem dados para salvar."); return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        nome = Path(__file__).resolve().parent / f"dados_malha_aberta_{ts}.csv"
        with open(nome, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['tempo_ms', 'pwm', 'alpha_rad'])
            for linha in dados:
                w.writerow([linha[0], linha[1], f"{linha[2]:.6f}"])
        self._ultimo_csv = str(nome)
        print(f"[OK] Salvo: {nome}  ({len(dados)} amostras)")
        self.txt_status.set_text(f"Salvo:\n{nome.name}")

    # -- Estimar G(s) -----------------------------------------
    def _estimar(self, event=None):
        csv_path = getattr(self, '_ultimo_csv', None)
        if not csv_path:
            self._salvar_csv()
            csv_path = getattr(self, '_ultimo_csv', None)
        if not csv_path:
            print("[AVISO] Nada para estimar."); return
        print(f"[INFO] Rodando estimar_tf.py em {csv_path} ...")
        try:
            import subprocess
            subprocess.Popen([sys.executable, "estimar_tf.py", csv_path])
        except Exception as e:
            print(f"[ERRO] Nao foi possivel rodar estimar_tf.py: {e}")
            print("       Rode manualmente: python estimar_tf.py <csv>")

    def _on_close(self, event):
        self._teste_abortar = True
        self._enviar("B")
        self.leitor.parar()
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass

    def run(self):
        plt.show()


# -- Entry point -----------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Monitor em tempo real do pendulo de Furuta.")
    parser.add_argument('--port', help='Porta serial (ex: COM3)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    args = parser.parse_args()

    porta = detectar_porta(args.port)
    print(f"[INFO] Conectando em {porta} @ {args.baud} baud ...")
    ser = serial.Serial(porta, args.baud, timeout=1)
    time.sleep(2.5)
    ser.reset_input_buffer()
    print("[INFO] Conectado. Abrindo interface...")
    print("[DICA] Manual: digite o degrau e clique Degrau +/-.")
    print("[DICA] Ou ajuste os campos e clique 'Teste varredura' ou 'Teste aleatorio'.")

    app = MonitorApp(ser)
    app.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[AVISO] Encerrado pelo usuario.")
    except Exception as e:
        print(f"[ERRO] {e}")
        sys.exit(1)