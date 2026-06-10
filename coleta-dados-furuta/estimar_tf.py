"""
============================================================
 estimar_tf.py  (versão corrigida)
 PÊNDULO DE FURUTA — Estimativa da Função de Transferência
 Entrega 2 — Introdução a Sistemas de Controle — UFMT
============================================================

COMO USAR:
  python estimar_tf.py dados_malha_aberta_YYYYMMDD_HHMMSS.csv

  Se nenhum arquivo for passado, abre um seletor de arquivos.

DEPENDÊNCIAS:
  pip install numpy scipy matplotlib pandas

O QUE MUDOU EM RELAÇÃO À VERSÃO ANTERIOR:
  1. O ponto de operação agora é U_op = 0 (motor desligado), não o
     maior PWM do ensaio. Para identificação a partir do repouso isso
     é o fisicamente correto.
  2. O modelo de 1ª ordem foi REMOVIDO: um pêndulo é um sistema
     oscilatório de 2ª ordem e a 1ª ordem nunca representa a oscilação.
  3. A estimativa de wn e zeta usa o método clássico do DECREMENTO
     LOGARITMICO aplicado ao trecho de RESPOSTA LIVRE (PWM = 0),
     que e onde a dinamica pura do sistema aparece.
  4. Kp e estimado pelo valor de repouso (com ressalva sobre a
     duracao do ensaio).
============================================================
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use backend sem display
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.signal import find_peaks
from pathlib import Path


def carregar_csv(caminho):
    df = pd.read_csv(caminho)
    df.columns = [c.strip() for c in df.columns]
    rename = {}
    for col in df.columns:
        low = col.lower()
        if 'tempo' in low or 'time' in low:
            rename[col] = 'tempo_ms'
        elif 'pwm' in low:
            rename[col] = 'pwm'
        elif 'alpha' in low and 'dot' not in low:
            rename[col] = 'alpha_rad'
    df.rename(columns=rename, inplace=True)
    df['t_s'] = (df['tempo_ms'] - df['tempo_ms'].iloc[0]) / 1000.0
    df['grupo'] = (df['pwm'] != df['pwm'].shift()).cumsum()
    return df


def segmentos_por_pwm(df, pwm_val):
    blocos = []
    for _, grp in df.groupby('grupo'):
        if grp['pwm'].iloc[0] == pwm_val:
            blocos.append(grp.reset_index(drop=True))
    return blocos


def resposta_livre_2a(t, y0, dy0, wn, zeta):
    wd = wn * np.sqrt(1 - zeta**2)
    A = y0
    B = (dy0 + zeta * wn * y0) / wd
    return np.exp(-zeta * wn * t) * (A * np.cos(wd * t) + B * np.sin(wd * t))


def estimar_parametros(df):
    U_op = 0.0
    blocos_zero = segmentos_por_pwm(df, 0)
    if not blocos_zero:
        raise RuntimeError("Nenhum bloco com PWM=0 encontrado.")

    bloco_final = blocos_zero[-1]
    Y_op = bloco_final['alpha_rad'].tail(max(5, len(bloco_final) // 3)).mean()

    bloco_livre = blocos_zero[0]
    t_bl = (bloco_livre['tempo_ms'] - bloco_livre['tempo_ms'].iloc[0]).values / 1000.0
    y_bl = (bloco_livre['alpha_rad'] - Y_op).values

    picos, _ = find_peaks(y_bl, height=0.02)
    vales, _ = find_peaks(-y_bl, height=0.02)
    extremos = sorted(list(picos) + list(vales))
    if len(extremos) < 2:
        extremos = sorted([int(np.argmax(y_bl)), int(np.argmin(y_bl))])

    i_e1, i_e2 = extremos[0], extremos[1]
    Td_half = abs(t_bl[i_e2] - t_bl[i_e1])
    Td = 2 * Td_half
    wd = 2 * np.pi / Td

    A1 = abs(y_bl[i_e1])
    A2 = abs(y_bl[i_e2])
    ratio = A2 / A1 if A1 != 0 else 1.0
    ln_ratio = -np.log(max(ratio, 1e-6))
    delta = 2 * ln_ratio
    zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
    wn = wd / np.sqrt(1 - zeta**2)

    pwm_unicos = sorted(df['pwm'].unique(), key=abs, reverse=True)
    nao_zero = [v for v in pwm_unicos if v != 0]
    dU = float(nao_zero[0]) if nao_zero else 1.0
    dY_ss_amp = abs(bloco_final['alpha_rad'].mean() - Y_op)
    Kp = dY_ss_amp / abs(dU) if dU != 0 else float('nan')

    return {
        'U_op': U_op, 'Y_op': Y_op, 'wd': wd, 'wn': wn, 'zeta': zeta,
        'Kp': Kp, 'dU': dU, 'Td': Td, 't_natural': 2 * np.pi / wn,
        'bloco_livre': (t_bl, y_bl), 'extremos': (i_e1, i_e2, A1, A2),
    }


def analisar(caminho_csv):
    print(f"\n[INFO] Lendo: {caminho_csv}")
    df = carregar_csv(caminho_csv)
    print(f"[INFO] {len(df)} amostras | duracao {df['t_s'].iloc[-1]:.2f} s | "
          f"Ts medio {df['tempo_ms'].diff().dropna().mean():.1f} ms")

    par = estimar_parametros(df)
    U_op, Y_op = par['U_op'], par['Y_op']
    wd, wn, zeta, Kp, dU = par['wd'], par['wn'], par['zeta'], par['Kp'], par['dU']

    print("\n" + "=" * 56)
    print("  PARAMETROS ESTIMADOS - MODELO DE 2a ORDEM")
    print("=" * 56)
    print(f"  Ponto de operacao:  U_op = {U_op:.0f}   Y_op = {Y_op:.5f} rad ({np.degrees(Y_op):.2f} graus)")
    print(f"  wd (amortecida)  = {wd:.4f} rad/s   (Td = {par['Td']:.4f} s)")
    print(f"  zeta             = {zeta:.4f}")
    print(f"  wn (natural)     = {wn:.4f} rad/s   (T = {par['t_natural']:.4f} s)")
    print(f"  Kp (ganho)       = {Kp:.6f} rad/PWM  [estimativa - ver ressalva]")
    print("-" * 56)
    num = Kp * wn**2
    print(f"  G(s) = {Kp:.5f}*{wn:.3f}^2 / (s^2 + 2*{zeta:.4f}*{wn:.3f}*s + {wn:.3f}^2)")
    print(f"       = {num:.5f} / (s^2 + {2*zeta*wn:.4f}*s + {wn**2:.4f})")
    print("=" * 56)
    print("  RESSALVA: se os degraus forem curtos (<5 s), o pendulo nao")
    print("  atinge regime permanente e Kp e apenas aproximado. wn e zeta,")
    print("  porem, sao confiaveis pois vem da oscilacao livre.")
    print("=" * 56)

    t_all = df['t_s'].values
    u_all = df['pwm'].values.astype(float)
    y_all = df['alpha_rad'].values
    t_bl, y_bl = par['bloco_livre']
    i_e1, i_e2, A1, A2 = par['extremos']

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Pendulo de Furuta - Ensaio em Malha Aberta\n"
                 "Estimativa da Funcao de Transferencia (2a ordem)",
                 fontsize=13, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.50, wspace=0.38)

    ax1 = fig.add_subplot(gs[0, :])
    ax1.step(t_all, u_all, where='post', color='royalblue', lw=1.8, label='PWM - U(t)')
    ax1.axhline(U_op, color='gray', ls='--', lw=1.2, label=f'U_op = {U_op:.0f}')
    ax1.set_ylabel('PWM'); ax1.set_xlabel('Tempo (s)')
    ax1.set_title('Entrada U(t)'); ax1.legend(fontsize=8); ax1.grid(alpha=0.3)

    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t_all, np.degrees(y_all), color='darkorange', lw=1.4, label='alpha medido')
    ax2.axhline(np.degrees(Y_op), color='gray', ls='--', lw=1.2, label=f'Y_op ~ {np.degrees(Y_op):.2f} graus')
    ax2.set_ylabel('alpha (graus)'); ax2.set_xlabel('Tempo (s)')
    ax2.set_title('Saida Y(t)'); ax2.legend(fontsize=8); ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(gs[2, :])
    ax3.plot(t_bl, np.degrees(y_bl), 'o', color='darkorange', ms=4, alpha=0.8,
             label='dY experimental (resposta livre)')
    if len(t_bl) > 2:
        Ts = np.mean(np.diff(t_bl))
        y0 = y_bl[0]; dy0 = (y_bl[1] - y_bl[0]) / Ts
        t_fine = np.linspace(0, t_bl[-1], 400)
        y_mod = resposta_livre_2a(t_fine, y0, dy0, wn, zeta)
        ax3.plot(t_fine, np.degrees(y_mod), '-', color='green', lw=2.2,
                 label=f'Modelo 2a ordem: wn={wn:.2f}, zeta={zeta:.3f}')
    ax3.annotate(f'A1={np.degrees(A1):.1f}', xy=(t_bl[i_e1], np.degrees(y_bl[i_e1])),
                 xytext=(t_bl[i_e1]+0.05, np.degrees(y_bl[i_e1])-8),
                 arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=8)
    ax3.annotate(f'A2={np.degrees(A2):.1f}', xy=(t_bl[i_e2], np.degrees(y_bl[i_e2])),
                 xytext=(t_bl[i_e2]+0.05, np.degrees(y_bl[i_e2])+5),
                 arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=8)
    ax3.axhline(0, color='gray', ls='--', lw=1)
    ax3.set_title('Resposta livre (PWM=0) - estimativa de wn e zeta por decremento logaritmico')
    ax3.set_xlabel('Tempo (s)'); ax3.set_ylabel('dalpha (graus)')
    ax3.legend(fontsize=8); ax3.grid(alpha=0.3)

    txt = (f'G(s) = {num:.4f}\n'
           f'       ----------------\n'
           f'       s^2+{2*zeta*wn:.3f}s+{wn**2:.2f}\n\n'
           f'wn = {wn:.3f} rad/s\nzeta = {zeta:.4f}\nKp = {Kp:.6f}')
    fig.text(0.78, 0.01, txt, fontsize=8, family='monospace',
             bbox=dict(facecolor='lightyellow', alpha=0.85, edgecolor='gray'))

    base = Path(caminho_csv).stem
    nome_png = Path(caminho_csv).parent / f"{base}_estimativa_tf.png"
    fig.savefig(nome_png, dpi=150, bbox_inches='tight')
    print(f"\n[OK] Grafico salvo em: {nome_png}")
    # plt.show()  # Removido para ambiente sem display


if __name__ == '__main__':
    if len(sys.argv) > 1:
        arquivo = sys.argv[1]
    else:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk(); root.withdraw()
            arquivo = filedialog.askopenfilename(
                title="Selecione o CSV de dados",
                filetypes=[("CSV", "*.csv"), ("Todos", "*.*")])
            if not arquivo:
                print("[ERRO] Nenhum arquivo selecionado."); sys.exit(1)
        except Exception:
            print("Uso: python estimar_tf.py <arquivo.csv>"); sys.exit(1)

    if not os.path.isfile(arquivo):
        print(f"[ERRO] Arquivo nao encontrado: {arquivo}"); sys.exit(1)
    analisar(arquivo)
