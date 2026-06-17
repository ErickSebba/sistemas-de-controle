"""
============================================================
 monitor_tempo_real.py
 PÊNDULO DE FURUTA — Monitor em tempo real com controle interativo
                     + Comparação com Transfer Function identificada
 Entrega 2 — Introdução a Sistemas de Controle — UFMT
============================================================

TRANSFER FUNCTION IDENTIFICADA (parâmetros físicos):
    Kp    = 2.8968e-03
    zeta  = 0.0743
    wn    = 9.1069 rad/s

    Forma padrão de 2ª ordem:
                       Kp · wn²
    H(s) = ──────────────────────────────
             s² + 2·zeta·wn·s + wn²

    Expandindo:
        Numerador  : Kp * wn² ≈ 0.24042
        Denominador: s² + 1.35327 s + 82.9357

O QUE FAZ:
  - Conecta no Arduino (sketch furuta_interativo.ino)
  - Plota AO VIVO: PWM, alpha real (sensor) e alpha simulado pela TF
  - Painel de erro (real − TF) com RMSE e MAE em tempo real
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
  pip install pyserial matplotlib numpy scipy

INTERFACE:
  - Campo "Degrau (PWM)": digite o valor (ex: 150)
  - Botão "Degrau +"     → envia PWM = +valor
  - Botão "Degrau -"     → envia PWM = -valor
  - Botão "Freio (0)"    → envia PWM = 0
  - Botão "Zerar α"      → zera o encoder
  - Botão "Salvar CSV"   → grava os dados coletados até agora
  - Botão "Estimar G(s)" → roda estimar_tf.py no CSV salvo
  - Botão "Limpar tela"  → limpa os buffers de plotagem

  TESTE AUTOMATIZADO (varredura +/-):
  - PWM ini / passo / PWM fim → níveis de PWM percorridos
  - rep min / rep max (s)     → descanso aleatório entre pulsos
  - pulso (ms)                → duração do pulso no PWM inicial
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
import warnings
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy import signal

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox


# ─────────────────────────────────────────────────────────────
# CONFIGURAÇÕES GERAIS
# ─────────────────────────────────────────────────────────────
BAUD_RATE = 115200
JANELA_S  = 12.0       # segundos visíveis na janela deslizante
MAX_PTS   = 5000       # máximo de pontos no buffer de plotagem

# ─────────────────────────────────────────────────────────────
# TRANSFER FUNCTION  (forma padrão de 2ª ordem)
#
#              Kp · wn²
#  H(s) = ─────────────────────────
#           s² + 2·ζ·wn·s + wn²
#
#  Kp   = 2.8968e-03
#  zeta = 0.0743
#  wn   = 9.1069 rad/s
# ─────────────────────────────────────────────────────────────
KP   = 2.8968e-03
ZETA = 0.0743
WN   = 9.1069

_b0 = KP * WN**2          # ≈ 0.24042
_a1 = 2.0 * ZETA * WN     # ≈ 1.35327
_a0 = WN**2               # ≈ 82.9357

TF_NUM = [_b0]
TF_DEN = [1.0, _a1, _a0]

sys_tf = signal.TransferFunction(TF_NUM, TF_DEN)

# -----------------------------------------------------------------
# TF1 (transfer function identified previously, for comparison)
#   H1(s) = (-0.001444 s + 0.143) / (s^2 + 1.23 s + 77.9)
# -----------------------------------------------------------------
TF1_NUM = [-0.001444, 0.143]
TF1_DEN = [1.0, 1.23, 77.9]
sys_tf1 = signal.TransferFunction(TF1_NUM, TF1_DEN)


def simular_tf(t_vec: np.ndarray, u_vec: np.ndarray) -> np.ndarray:
    """Simula a TF com o PWM real como entrada (scipy lsim).
    Retorna alpha simulado em radianos."""
    if len(t_vec) < 2:
        return np.zeros_like(t_vec)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        _, y_out, _ = signal.lsim(sys_tf, u_vec, t_vec)
    return y_out


def simular_tf_generic(sys_model, t_vec: np.ndarray, u_vec: np.ndarray) -> np.ndarray:
    """Simula o sistema de transferência fornecido com `lsim`.
    `sys_model` deve ser um `TransferFunction` do scipy."""
    if len(t_vec) < 2:
        return np.zeros_like(t_vec)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        _, y_out, _ = signal.lsim(sys_model, u_vec, t_vec)
    return y_out


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
        self.t     = deque(maxlen=MAX_PTS)
        self.pwm   = deque(maxlen=MAX_PTS)
        self.alpha = deque(maxlen=MAX_PTS)   # graus, para o visual
        self.todos = []
        self.t0_ms = None
        self.lock  = threading.Lock()

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
                alpha    = float(partes[2])   # radianos
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


# ── Aplicação principal ───────────────────────────────────────

class MonitorApp:
    def __init__(self, ser):
        self.ser      = ser
        self.ser_lock = threading.Lock()
        self.leitor   = LeitorSerial(ser)
        self.leitor.start()
        self.degrau = 150

        # ── Parâmetros do teste automatizado ─────────────────
        self.pwm_ini         = 100
        self.pwm_passo       = 50
        self.pwm_fim         = 250
        self.rep_min         = 6.0
        self.rep_max         = 7.0
        self.t_pulso_base_ms = 600
        self.t_pulso_min_ms  = 50
        self._teste_rodando  = False
        self._teste_abortar  = False
        self._teste_info     = ""

        # ── Layout: 3 painéis ─────────────────────────────────
        self.fig, (self.ax_u, self.ax_y, self.ax_err) = plt.subplots(
            3, 1, figsize=(12, 10), sharex=True)
        self.fig.subplots_adjust(bottom=0.40, hspace=0.38)
        self.fig.suptitle(
            "Pêndulo de Furuta — Monitor em Tempo Real  |  "
            r"$H(s)=\frac{K_p\,\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}$"
            f"   Kp={KP:.4e}  ζ={ZETA}  ωn={WN} rad/s",
            fontsize=10, fontweight='bold'
        )

        # Painel 1 — PWM
        self.linha_u, = self.ax_u.step([], [], where='post',
                                        color='royalblue', lw=1.8)
        self.ax_u.set_ylabel("PWM")
        self.ax_u.set_ylim(-270, 270)
        self.ax_u.set_title("Entrada U(t) — PWM aplicado")
        self.ax_u.grid(alpha=0.3)

        # Painel 2 — alpha real vs TF
        self.linha_y,    = self.ax_y.plot([], [], color='darkorange',
                           lw=1.5, label="Real (sensor)")
        # TF2 (modelo do usuário: KP/ZETA/WN)
        self.linha_sim,  = self.ax_y.plot([], [], color='purple', lw=1.5,
                           linestyle='--', label="TF2 (simulado)")
        # TF1 (transfer function para comparação)
        self.linha_sim1, = self.ax_y.plot([], [], color='seagreen', lw=1.5,
                           linestyle='-.', label="TF1 (simulado)")
        self.ax_y.set_ylabel("α (graus)")
        self.ax_y.set_title("Saída Y(t) — ângulo do pêndulo  (real  vs  TF)")
        self.ax_y.legend(loc='upper right', fontsize=8)
        self.ax_y.grid(alpha=0.3)

        # Painel 3 — erro
        self.linha_err, = self.ax_err.plot([], [], color='crimson',
                                            lw=1.2, label="Erro: real − TF")
        self.ax_err.axhline(0, color='gray', lw=0.8, linestyle=':')
        self.ax_err.set_ylabel("Erro (graus)")
        self.ax_err.set_xlabel("Tempo (s)")
        self.ax_err.set_title("Erro entre sensor e TF")
        self.ax_err.legend(loc='upper right', fontsize=8)
        self.ax_err.grid(alpha=0.3)

        self.txt_metricas = self.ax_err.text(
            0.01, 0.95, "", transform=self.ax_err.transAxes,
            fontsize=8, va='top',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', alpha=0.85)
        )

        self._criar_controles()

        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._atualizar_plot)
        self.timer.start()

        self.fig.canvas.mpl_connect('close_event', self._on_close)

    # ── Controles ─────────────────────────────────────────────
    def _criar_controles(self):
        # Linha A: degrau manual
        ax_box = self.fig.add_axes([0.11, 0.325, 0.07, 0.040])
        self.box_degrau = TextBox(ax_box, "Degrau  ", initial=str(self.degrau))
        self.box_degrau.on_submit(self._set_degrau)

        ax_mais  = self.fig.add_axes([0.27, 0.325, 0.08, 0.040])
        ax_menos = self.fig.add_axes([0.36, 0.325, 0.08, 0.040])
        ax_freio = self.fig.add_axes([0.45, 0.325, 0.08, 0.040])
        ax_zero  = self.fig.add_axes([0.54, 0.325, 0.08, 0.040])

        self.btn_mais   = Button(ax_mais,  "Degrau +",  color='#c8e6c9')
        self.btn_menos  = Button(ax_menos, "Degrau -",  color='#ffcdd2')
        self.btn_freio  = Button(ax_freio, "Freio (0)", color='#fff9c4')
        self.btn_zero   = Button(ax_zero,  "Zerar α",   color='#bbdefb')

        self.btn_mais.on_clicked(lambda e: self._enviar(f"P{self.degrau}"))
        self.btn_menos.on_clicked(lambda e: self._enviar(f"P{-self.degrau}"))
        self.btn_freio.on_clicked(lambda e: self._enviar("B"))
        self.btn_zero.on_clicked(lambda e: self._enviar("Z"))

        # Linha B: arquivo
        ax_salvar = self.fig.add_axes([0.11, 0.265, 0.11, 0.040])
        ax_estim  = self.fig.add_axes([0.23, 0.265, 0.11, 0.040])
        ax_limpar = self.fig.add_axes([0.35, 0.265, 0.11, 0.040])

        self.btn_salvar = Button(ax_salvar, "Salvar CSV",    color='#d1c4e9')
        self.btn_estim  = Button(ax_estim,  "Estimar G(s)",  color='#b2dfdb')
        self.btn_limpar = Button(ax_limpar, "Limpar tela",   color='#eeeeee')

        self.btn_salvar.on_clicked(self._salvar_csv)
        self.btn_estim.on_clicked(self._estimar)
        self.btn_limpar.on_clicked(self._limpar)

        # Cabeçalho do teste automatizado
        self.fig.text(0.11, 0.225, "TESTE AUTOMATIZADO (varredura +/−)",
                      fontsize=9, fontweight='bold', color='#444')

        # Linha C: parâmetros de PWM
        ax_pini   = self.fig.add_axes([0.11, 0.155, 0.055, 0.040])
        ax_ppasso = self.fig.add_axes([0.29, 0.155, 0.055, 0.040])
        ax_pfim   = self.fig.add_axes([0.47, 0.155, 0.055, 0.040])
        self.box_pwm_ini   = TextBox(ax_pini,   "PWM ini  ", initial=str(self.pwm_ini))
        self.box_pwm_passo = TextBox(ax_ppasso, "passo  ",   initial=str(self.pwm_passo))
        self.box_pwm_fim   = TextBox(ax_pfim,   "PWM fim  ", initial=str(self.pwm_fim))
        self.box_pwm_ini.on_submit(  lambda t: self._set_param('pwm_ini',   t, int))
        self.box_pwm_passo.on_submit(lambda t: self._set_param('pwm_passo', t, int))
        self.box_pwm_fim.on_submit(  lambda t: self._set_param('pwm_fim',   t, int))

        # Linha D: tempos
        ax_rmin  = self.fig.add_axes([0.11, 0.085, 0.055, 0.040])
        ax_rmax  = self.fig.add_axes([0.29, 0.085, 0.055, 0.040])
        ax_pulso = self.fig.add_axes([0.47, 0.085, 0.055, 0.040])
        self.box_rep_min = TextBox(ax_rmin,  "rep min (s)  ", initial=str(self.rep_min))
        self.box_rep_max = TextBox(ax_rmax,  "rep max (s)  ", initial=str(self.rep_max))
        self.box_pulso   = TextBox(ax_pulso, "pulso (ms)  ",  initial=str(self.t_pulso_base_ms))
        self.box_rep_min.on_submit(lambda t: self._set_param('rep_min',         t, float))
        self.box_rep_max.on_submit(lambda t: self._set_param('rep_max',         t, float))
        self.box_pulso.on_submit(  lambda t: self._set_param('t_pulso_base_ms', t, float))

        # Botões do teste
        ax_tstart = self.fig.add_axes([0.62, 0.155, 0.14, 0.045])
        ax_tstop  = self.fig.add_axes([0.62, 0.085, 0.14, 0.045])
        self.btn_teste_start = Button(ax_tstart, "Iniciar teste", color='#a5d6a7')
        self.btn_teste_stop  = Button(ax_tstop,  "Parar teste",   color='#ef9a9a')
        self.btn_teste_start.on_clicked(self._iniciar_teste)
        self.btn_teste_stop.on_clicked(self._parar_teste)

        self.txt_status = self.fig.text(
            0.80, 0.155, "", fontsize=8,
            family='monospace', color='navy')

    # ── Setters ───────────────────────────────────────────────
    def _set_degrau(self, txt):
        try:
            self.degrau = int(float(txt))
            print(f"[INFO] degrau = {self.degrau}")
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
        pwm  = max(1, abs(int(pwm)))
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
        fim = time.time() + dur
        while time.time() < fim:
            if self._teste_abortar:
                return
            time.sleep(0.02)

    def _loop_teste(self):
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
                    self._enviar(f"P{valor}")
                    time.sleep(t_pulso)
                    self._enviar("B")
                    self._descanso_interrompivel(
                        random.uniform(self.rep_min, self.rep_max))
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
            t     = np.array(self.leitor.t)
            u     = np.array(self.leitor.pwm)
            y_deg = np.array(self.leitor.alpha)

        if len(t) < 2:
            return

        # Simula TF2 com PWM real → converte rad → graus
        y_sim_rad  = simular_tf(t, u)
        y_sim_deg  = np.degrees(y_sim_rad)
        erro_deg   = y_deg - y_sim_deg

        # Simula TF1 (comparativa)
        y_sim1_rad = simular_tf_generic(sys_tf1, t, u)
        y_sim1_deg = np.degrees(y_sim1_rad)
        erro1_deg  = y_deg - y_sim1_deg

        self.linha_u.set_data(t, u)
        self.linha_y.set_data(t, y_deg)
        self.linha_sim.set_data(t, y_sim_deg)
        self.linha_sim1.set_data(t, y_sim1_deg)
        self.linha_err.set_data(t, erro_deg)

        # Janela deslizante
        t_max = t[-1]
        t_min = max(0.0, t_max - JANELA_S)
        for ax in (self.ax_u, self.ax_y, self.ax_err):
            ax.set_xlim(t_min, t_max + 0.5)

        # Auto-escala Y — alpha (cobre real + simulado)
        vis = t >= t_min
        if vis.any():
            ymin = min(y_deg[vis].min(), y_sim_deg[vis].min())
            ymax = max(y_deg[vis].max(), y_sim_deg[vis].max())
            margem = max(5.0, 0.1 * (ymax - ymin))
            self.ax_y.set_ylim(ymin - margem, ymax + margem)

        # Auto-escala Y — erro
        if vis.any() and len(erro_deg[vis]):
            emin, emax = erro_deg[vis].min(), erro_deg[vis].max()
            em = max(1.0, 0.15 * (emax - emin))
            self.ax_err.set_ylim(emin - em, emax + em)

        # Métricas (ambas TFs)
        rmse  = np.sqrt(np.mean(erro_deg ** 2))
        mae   = np.mean(np.abs(erro_deg))
        rmse1 = np.sqrt(np.mean(erro1_deg ** 2))
        mae1  = np.mean(np.abs(erro1_deg))
        self.txt_metricas.set_text(
            f"TF2: RMSE={rmse:.2f}°, MAE={mae:.2f}°\nTF1: RMSE={rmse1:.2f}°, MAE={mae1:.2f}°"
        )

        # Status
        status = (
            f"PWM:    {int(u[-1]):+4d}\n"
            f"α real: {y_deg[-1]:+6.1f}°\n"
            f"α TF2:  {y_sim_deg[-1]:+6.1f}°\n"
            f"α TF1:  {y_sim1_deg[-1]:+6.1f}°\n"
            f"amostras: {len(self.leitor.todos)}"
        )
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
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
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


# ── Entry point ───────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Monitor em tempo real do pendulo de Furuta + comparacao TF.")
    parser.add_argument('--port', help='Porta serial (ex: COM3 ou /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    args = parser.parse_args()

    porta = detectar_porta(args.port)
    print(f"[INFO] Conectando em {porta} @ {args.baud} baud ...")
    ser = serial.Serial(porta, args.baud, timeout=1)
    time.sleep(2.5)
    ser.reset_input_buffer()
    print("[INFO] Conectado. Abrindo interface...")
    print("[DICA] Digite o tamanho do degrau, depois clique 'Degrau +' ou 'Degrau -'.")
    print("[DICA] Ou ajuste o painel de TESTE e clique 'Iniciar teste'.")
    print(f"[TF]   H(s) = {_b0:.5f} / (s^2 + {_a1:.5f}*s + {_a0:.4f})")
    print(f"       Kp={KP}  zeta={ZETA}  wn={WN} rad/s")

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
