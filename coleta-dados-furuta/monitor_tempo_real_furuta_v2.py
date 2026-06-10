"""
============================================================
 monitor_tempo_real.py
 PÊNDULO DE FURUTA — Monitor em tempo real com controle interativo
 Entrega 2 — Introdução a Sistemas de Controle — UFMT
============================================================

O QUE FAZ:
  - Conecta no Arduino (sketch furuta_interativo.ino)
  - Plota AO VIVO o PWM aplicado e o ângulo do pêndulo
  - Permite ESCOLHER o tamanho do degrau e APLICAR na hora
  - Botões: aplicar +degrau, -degrau, freio, zerar encoder
  - TESTE AUTOMATIZADO: varredura de pulsos +/- com descanso
  - Grava tudo num CSV (compatível com estimar_tf.py)
  - Botão para estimar G(s) na hora a partir do que foi gravado

COMO USAR:
  1. Carregue o sketch furuta_interativo.ino no Arduino Mega.
  2. Feche o Monitor Serial da Arduino IDE.
  3. Execute:
       python monitor_tempo_real.py
     ou com porta explícita:
       python monitor_tempo_real.py --port COM3

DEPENDÊNCIAS:
  pip install pyserial matplotlib numpy

INTERFACE:
  - Campo "Degrau (PWM)": digite o valor (ex: 150)
  - Botão "Degrau +"  → envia PWM = +valor
  - Botão "Degrau -"  → envia PWM = -valor
  - Botão "Freio (0)" → envia PWM = 0
  - Botão "Zerar α"   → zera o encoder
  - Botão "Salvar CSV"→ grava os dados coletados até agora
  - Botão "Estimar G(s)" → roda estimar_tf.py no CSV salvo

  TESTE AUTOMATIZADO (varredura +/-):
  - PWM ini / passo / PWM fim → níveis de PWM percorridos
  - rep min / rep max (s)     → descanso aleatório entre pulsos
  - pulso (ms)                → duração do pulso NO PWM INICIAL
                                (encolhe sozinho conforme o PWM cresce)
  - Botão "Iniciar teste" / "Parar teste"
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
JANELA_S  = 12.0          # segundos visíveis no gráfico (janela deslizante)
MAX_PTS   = 5000          # máximo de pontos no buffer de plotagem


# ── Detecção de porta ─────────────────────────────────────────

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


# ── Leitor serial em thread separada ──────────────────────────

class LeitorSerial(threading.Thread):
    """Lê o Arduino continuamente e armazena os dados num buffer."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.rodando = True
        # Buffers para plotagem (deque com limite)
        self.t = deque(maxlen=MAX_PTS)
        self.pwm = deque(maxlen=MAX_PTS)
        self.alpha = deque(maxlen=MAX_PTS)
        # Buffer completo para gravar em CSV
        self.todos = []   # lista de (tempo_ms, pwm, alpha_rad)
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
                self.alpha.append(np.degrees(alpha))   # grava em graus p/ visual
                self.todos.append((int(tempo_ms), int(pwm_val), alpha))

    def parar(self):
        self.rodando = False


# ── Aplicação principal ───────────────────────────────────────

class MonitorApp:
    def __init__(self, ser):
        self.ser = ser
        self.ser_lock = threading.Lock()      # protege escritas concorrentes
        self.leitor = LeitorSerial(ser)
        self.leitor.start()
        self.degrau = 150     # valor padrão do degrau manual

        # ── Parâmetros do teste automatizado ─────────────────
        self.pwm_ini   = 100      # PWM inicial da varredura
        self.pwm_passo = 50       # incremento de PWM por nível
        self.pwm_fim   = 250      # PWM final (analogWrite vai até 255)
        self.rep_min   = 6.0      # descanso mínimo (s)
        self.rep_max   = 7.0      # descanso máximo (s)
        self.t_pulso_base_ms = 600   # pulso NO PWM inicial (ms)
        self.t_pulso_min_ms  = 50   # piso de segurança do pulso (ms)
        self._teste_rodando = False
        self._teste_abortar = False
        self._teste_info    = ""

        # ── Layout da figura ──────────────────────────────────
        self.fig, (self.ax_u, self.ax_y) = plt.subplots(
            2, 1, figsize=(11, 8.6), sharex=True)
        self.fig.subplots_adjust(bottom=0.40, hspace=0.25)
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

        # Timer de atualização do gráfico (~20 fps)
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._atualizar_plot)
        self.timer.start()

        self.fig.canvas.mpl_connect('close_event', self._on_close)

    # ── Controles (botões e campos de texto) ──────────────────
    def _criar_controles(self):
        # ---- Linha A: degrau manual ----
        ax_box = self.fig.add_axes([0.11, 0.32, 0.07, 0.045])
        self.box_degrau = TextBox(ax_box, "Degrau  ", initial=str(self.degrau))
        self.box_degrau.on_submit(self._set_degrau)

        ax_mais  = self.fig.add_axes([0.27, 0.32, 0.08, 0.045])
        ax_menos = self.fig.add_axes([0.36, 0.32, 0.08, 0.045])
        ax_freio = self.fig.add_axes([0.45, 0.32, 0.08, 0.045])
        ax_zero  = self.fig.add_axes([0.54, 0.32, 0.08, 0.045])

        self.btn_mais   = Button(ax_mais,  "Degrau +",  color='#c8e6c9')
        self.btn_menos  = Button(ax_menos, "Degrau -",  color='#ffcdd2')
        self.btn_freio  = Button(ax_freio, "Freio (0)", color='#fff9c4')
        self.btn_zero   = Button(ax_zero,  "Zerar a",   color='#bbdefb')

        self.btn_mais.on_clicked(lambda e: self._enviar(f"P{self.degrau}"))
        self.btn_menos.on_clicked(lambda e: self._enviar(f"P{-self.degrau}"))
        self.btn_freio.on_clicked(lambda e: self._enviar("B"))
        self.btn_zero.on_clicked(lambda e: self._enviar("Z"))

        # ---- Linha B: arquivo ----
        ax_salvar = self.fig.add_axes([0.11, 0.25, 0.11, 0.045])
        ax_estim  = self.fig.add_axes([0.23, 0.25, 0.11, 0.045])
        ax_limpar = self.fig.add_axes([0.35, 0.25, 0.11, 0.045])

        self.btn_salvar = Button(ax_salvar, "Salvar CSV",    color='#d1c4e9')
        self.btn_estim  = Button(ax_estim,  "Estimar G(s)",  color='#b2dfdb')
        self.btn_limpar = Button(ax_limpar, "Limpar tela",   color='#eeeeee')

        self.btn_salvar.on_clicked(self._salvar_csv)
        self.btn_estim.on_clicked(self._estimar)
        self.btn_limpar.on_clicked(self._limpar)

        # ---- Painel do teste automatizado ----
        self.fig.text(0.11, 0.205, "TESTE AUTOMATIZADO (varredura +/-)",
                      fontsize=9, fontweight='bold', color='#444')

        # Linha C: parâmetros de PWM
        ax_pini   = self.fig.add_axes([0.11, 0.13, 0.05, 0.045])
        ax_ppasso = self.fig.add_axes([0.28, 0.13, 0.05, 0.045])
        ax_pfim   = self.fig.add_axes([0.45, 0.13, 0.05, 0.045])
        self.box_pwm_ini   = TextBox(ax_pini,   "PWM ini  ", initial=str(self.pwm_ini))
        self.box_pwm_passo = TextBox(ax_ppasso, "passo  ",   initial=str(self.pwm_passo))
        self.box_pwm_fim   = TextBox(ax_pfim,   "PWM fim  ", initial=str(self.pwm_fim))
        self.box_pwm_ini.on_submit(  lambda t: self._set_param('pwm_ini',   t, int))
        self.box_pwm_passo.on_submit(lambda t: self._set_param('pwm_passo', t, int))
        self.box_pwm_fim.on_submit(  lambda t: self._set_param('pwm_fim',   t, int))

        # Linha D: tempos
        ax_rmin  = self.fig.add_axes([0.11, 0.06, 0.05, 0.045])
        ax_rmax  = self.fig.add_axes([0.28, 0.06, 0.05, 0.045])
        ax_pulso = self.fig.add_axes([0.45, 0.06, 0.05, 0.045])
        self.box_rep_min = TextBox(ax_rmin,  "rep min (s)  ", initial=str(self.rep_min))
        self.box_rep_max = TextBox(ax_rmax,  "rep max (s)  ", initial=str(self.rep_max))
        self.box_pulso   = TextBox(ax_pulso, "pulso (ms)  ",  initial=str(self.t_pulso_base_ms))
        self.box_rep_min.on_submit(lambda t: self._set_param('rep_min', t, float))
        self.box_rep_max.on_submit(lambda t: self._set_param('rep_max', t, float))
        self.box_pulso.on_submit(  lambda t: self._set_param('t_pulso_base_ms', t, float))

        # Botões do teste
        ax_tstart = self.fig.add_axes([0.60, 0.13, 0.14, 0.05])
        ax_tstop  = self.fig.add_axes([0.60, 0.06, 0.14, 0.05])
        self.btn_teste_start = Button(ax_tstart, "Iniciar teste", color='#a5d6a7')
        self.btn_teste_stop  = Button(ax_tstop,  "Parar teste",   color='#ef9a9a')
        self.btn_teste_start.on_clicked(self._iniciar_teste)
        self.btn_teste_stop.on_clicked(self._parar_teste)

        # Texto de status
        self.txt_status = self.fig.text(0.78, 0.14, "", fontsize=9,
                                        family='monospace', color='navy')

    # ── Setters dos campos ────────────────────────────────────
    def _set_degrau(self, txt):
        try:
            self.degrau = int(float(txt))
            print(f"[INFO] Tamanho do degrau = {self.degrau}")
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

    # ── Teste automatizado ────────────────────────────────────
    def _tempo_pulso_s(self, pwm):
        """Duração do pulso (s) para um dado PWM.

        O pulso encolhe conforme o PWM cresce para manter o 'chute'
        aproximadamente constante. Como o deslocamento angular do braço
        ~ impulso ~ (PWM x tempo), fazemos:

            tempo = tempo_base * (pwm_ini / pwm)

        Assim PWM*tempo ~ const e o braço gira mais ou menos o mesmo
        tanto em todos os niveis -> nao enrosca o fio. Tem piso de
        seguranca para nunca virar pulso "zero".
        """
        pwm = max(1, abs(int(pwm)))
        t_ms = self.t_pulso_base_ms * (self.pwm_ini / pwm)
        t_ms = max(self.t_pulso_min_ms, t_ms)
        return t_ms / 1000.0

    def _iniciar_teste(self, event=None):
        if self._teste_rodando:
            print("[AVISO] Teste ja em andamento.")
            return
        if self.pwm_passo <= 0:
            print("[AVISO] 'passo' de PWM deve ser > 0.")
            return
        if self.pwm_fim < self.pwm_ini:
            print("[AVISO] 'PWM fim' deve ser >= 'PWM ini'.")
            return
        self._teste_abortar = False
        self._teste_rodando = True
        threading.Thread(target=self._loop_teste, daemon=True).start()
        print("[INFO] Teste automatizado iniciado.")

    def _parar_teste(self, event=None):
        if self._teste_rodando:
            self._teste_abortar = True
            self._enviar("B")
            print("[INFO] Parando teste (freio aplicado).")

    def _descanso_interrompivel(self, dur):
        """Espera 'dur' segundos podendo ser cortada pelo botao Parar."""
        fim = time.time() + dur
        while time.time() < fim:
            if self._teste_abortar:
                return
            time.sleep(0.02)

    def _loop_teste(self):
        """Percorre os niveis de PWM aplicando pulso DIR e ESQ em cada um,
        com descanso aleatorio (resposta livre) entre eles."""
        try:
            pwm = self.pwm_ini
            while pwm <= self.pwm_fim and not self._teste_abortar:
                for sinal, nome in ((+1, "DIR"), (-1, "ESQ")):
                    if self._teste_abortar:
                        break
                    valor   = sinal * pwm
                    t_pulso = self._tempo_pulso_s(pwm)
                    self._teste_info = (f"Teste: PWM {valor:+d} ({nome})\n"
                                        f"pulso {t_pulso*1000:.1f} ms")
                    print(f"[TESTE] PWM={valor:+d}  pulso={t_pulso*1000:.2f} ms")

                    self._enviar(f"P{valor}")     # chute
                    time.sleep(t_pulso)           # pulso curto
                    self._enviar("B")             # solta -> resposta livre

                    t_rep = random.uniform(self.rep_min, self.rep_max)
                    self._descanso_interrompivel(t_rep)
                pwm += self.pwm_passo
        finally:
            self._enviar("B")
            self._teste_rodando = False
            self._teste_info = ("Teste abortado." if self._teste_abortar
                                else "Teste finalizado.")
            print("[INFO] " + self._teste_info)

    # ── Atualização do gráfico ────────────────────────────────
    def _atualizar_plot(self):
        with self.leitor.lock:
            t = np.array(self.leitor.t)
            u = np.array(self.leitor.pwm)
            y = np.array(self.leitor.alpha)
        if len(t) < 2:
            return

        self.linha_u.set_data(t, u)
        self.linha_y.set_data(t, y)

        # Janela deslizante
        t_max = t[-1]
        t_min = max(0, t_max - JANELA_S)
        self.ax_u.set_xlim(t_min, t_max + 0.5)
        self.ax_y.set_xlim(t_min, t_max + 0.5)

        # Auto-escala do eixo Y de alpha
        vis = (t >= t_min)
        if vis.any():
            ymin, ymax = y[vis].min(), y[vis].max()
            margem = max(5, 0.1 * (ymax - ymin))
            self.ax_y.set_ylim(ymin - margem, ymax + margem)

        # Status
        status = (f"PWM atual: {int(u[-1]):+4d}\n"
                  f"alpha: {y[-1]:+6.2f} graus\n"
                  f"amostras: {len(self.leitor.todos)}")
        if self._teste_info:
            status += f"\n{self._teste_info}"
        self.txt_status.set_text(status)

        self.fig.canvas.draw_idle()

    # ── Salvar CSV ────────────────────────────────────────────
    def _salvar_csv(self, event=None):
        with self.leitor.lock:
            dados = list(self.leitor.todos)
        if not dados:
            print("[AVISO] Sem dados para salvar.")
            return
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

    # ── Estimar G(s) ──────────────────────────────────────────
    def _estimar(self, event=None):
        csv_path = getattr(self, '_ultimo_csv', None)
        if not csv_path:
            # salva automaticamente primeiro
            self._salvar_csv()
            csv_path = getattr(self, '_ultimo_csv', None)
        if not csv_path:
            print("[AVISO] Nada para estimar.")
            return
        print(f"[INFO] Rodando estimar_tf.py em {csv_path} ...")
        try:
            import subprocess
            subprocess.Popen([sys.executable, "estimar_tf.py", csv_path])
        except Exception as e:
            print(f"[ERRO] Nao foi possivel rodar estimar_tf.py: {e}")
            print("       Rode manualmente: python estimar_tf.py <csv>")

    def _on_close(self, event):
        self._teste_abortar = True   # corta o teste se estiver rodando
        self._enviar("B")            # freio ao fechar
        self.leitor.parar()
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass

    def run(self):
        plt.show()


# ── Entry point ───────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Monitor em tempo real do pendulo de Furuta.")
    parser.add_argument('--port', help='Porta serial (ex: COM3)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    args = parser.parse_args()

    porta = detectar_porta(args.port)
    print(f"[INFO] Conectando em {porta} @ {args.baud} baud ...")
    ser = serial.Serial(porta, args.baud, timeout=1)
    time.sleep(2.5)          # aguarda reset do Arduino
    ser.reset_input_buffer()
    print("[INFO] Conectado. Abrindo interface...")
    print("[DICA] Digite o tamanho do degrau, depois clique 'Degrau +' ou 'Degrau -'.")
    print("[DICA] Ou ajuste o painel de TESTE e clique 'Iniciar teste'.")

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