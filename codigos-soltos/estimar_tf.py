"""
============================================================
 estimar_tf.py
 PÊNDULO DE FURUTA — Estimativa da Função de Transferência
 Entrega 2 — Introdução a Sistemas de Controle — UFMT
============================================================

COMO USAR:
  python estimar_tf.py dados_malha_aberta_YYYYMMDD_HHMMSS.csv

  Se nenhum arquivo for passado, abre um seletor de arquivos.

DEPENDÊNCIAS:
  pip install numpy scipy matplotlib pandas

O QUE O SCRIPT FAZ:
  1. Lê o CSV gerado por coletar_dados.py
  2. Normaliza U e Y em torno do ponto de operação (desvio)
  3. Calcula a resposta ao degrau experimental (dY/dU)
  4. Oferece duas estimativas:
       a) 1ª ordem:  G(s) = Kp / (tau*s + 1)
       b) 2ª ordem:  G(s) = Kp*wn² / (s² + 2ζwn*s + wn²)
  5. Plota os dados experimentais + curvas ajustadas
  6. Salva gráfico como PNG e imprime os parâmetros estimados
============================================================
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.optimize import curve_fit
from scipy.signal import lti, step
from pathlib import Path


# ── Utilitários de resposta ao degrau ────────────────────────

def resposta_1a_ordem(t: np.ndarray, Kp: float, tau: float) -> np.ndarray:
    """y(t) = Kp*(1 - exp(-t/tau))  para t ≥ 0."""
    return Kp * (1.0 - np.exp(-t / tau))


def resposta_2a_ordem(t: np.ndarray, Kp: float, wn: float, zeta: float) -> np.ndarray:
    """
    Resposta ao degrau unitário de  G(s) = Kp*wn²/(s²+2ζwn*s+wn²).
    Usa scipy.signal.lti para cobrir sub/super-amortecido e criticamente amortecido.
    """
    sistema = lti([Kp * wn ** 2], [1, 2 * zeta * wn, wn ** 2])
    _, y = step(sistema, T=t)
    return y


# ── Estimativa por método analítico (1ª ordem) ───────────────

def estimar_1a_ordem_analitico(t: np.ndarray, y_norm: np.ndarray,
                                dU: float) -> tuple[float, float]:
    """
    Método dos 63%:
      Kp  = y_ss / dU
      tau = t onde y(t) ≈ 0.632 * y_ss
    """
    y_ss = y_norm[-1]               # valor de regime permanente
    Kp   = y_ss / dU if dU != 0 else float('nan')
    alvo = 0.6321 * y_ss
    idx  = np.searchsorted(y_norm, alvo)
    tau  = t[idx] if idx < len(t) else t[-1]
    return float(Kp), float(tau)


# ── Leitura e pré-processamento ───────────────────────────────

def carregar_csv(caminho: str) -> pd.DataFrame:
    df = pd.read_csv(caminho)
    df.columns = [c.strip() for c in df.columns]
    # Normaliza nomes de coluna
    rename = {}
    for col in df.columns:
        low = col.lower()
        if 'tempo' in low or 'time' in low:
            rename[col] = 'tempo_ms'
        elif 'pwm' in low:
            rename[col] = 'pwm'
        elif 'alpha_dot' in low or 'alphaDot' in low:
            rename[col] = 'alpha_dot'
        elif 'alpha' in low:
            rename[col] = 'alpha_rad'
    df.rename(columns=rename, inplace=True)
    return df


def segmentar_degraus(df: pd.DataFrame) -> list[pd.DataFrame]:
    """
    Divide o DataFrame em segmentos onde o PWM é constante.
    Retorna lista de DataFrames, um por degrau.
    """
    df = df.copy()
    df['grupo'] = (df['pwm'] != df['pwm'].shift()).cumsum()
    return [grp.reset_index(drop=True) for _, grp in df.groupby('grupo')]


# ── Análise principal ─────────────────────────────────────────

def analisar(caminho_csv: str):
    print(f"\n[INFO] Lendo: {caminho_csv}")
    df = carregar_csv(caminho_csv)

    # Tempo em segundos a partir do zero
    df['t_s'] = (df['tempo_ms'] - df['tempo_ms'].iloc[0]) / 1000.0

    print(f"[INFO] {len(df)} amostras | duração {df['t_s'].iloc[-1]:.1f} s")
    print(f"[INFO] PWM mín={df['pwm'].min()}  máx={df['pwm'].max()}")
    print(f"[INFO] α mín={df['alpha_rad'].min():.4f} rad  "
          f"máx={df['alpha_rad'].max():.4f} rad")

    # ── Ponto de operação: primeiro segmento ──────────────────
    segmentos = segmentar_degraus(df)
    seg0      = segmentos[0]
    U_op      = seg0['pwm'].iloc[0]
    Y_op      = seg0['alpha_rad'].mean()
    print(f"\n[INFO] Ponto de operação — U_op={U_op}  Y_op={Y_op:.6f} rad")

    # ── Variáveis de desvio ───────────────────────────────────
    df['dU'] = df['pwm']       - U_op
    df['dY'] = df['alpha_rad'] - Y_op

    # ── Seleciona o degrau positivo (maior excursão) ──────────
    # Pega o segmento com maior |dU| para estimativa
    melhor_seg = max(segmentos[1:],
                     key=lambda s: abs(s['pwm'].iloc[0] - U_op),
                     default=segmentos[1] if len(segmentos) > 1 else seg0)

    dU_val   = float(melhor_seg['pwm'].iloc[0] - U_op)
    t_seg    = (melhor_seg['t_s'] - melhor_seg['t_s'].iloc[0]).values
    y_seg    = (melhor_seg['alpha_rad'] - Y_op).values

    print(f"\n[INFO] Segmento usado na estimativa: dU = {dU_val:+.1f}")

    # ── Estimativa 1ª ordem ───────────────────────────────────
    Kp_1, tau_1 = estimar_1a_ordem_analitico(t_seg, y_seg, dU_val)

    # Refinamento por curve_fit
    try:
        popt_1, _ = curve_fit(
            lambda t, Kp, tau: resposta_1a_ordem(t, Kp * dU_val, tau),
            t_seg, y_seg,
            p0=[Kp_1, tau_1],
            maxfev=5000,
            bounds=([-np.inf, 1e-4], [np.inf, np.inf])
        )
        Kp_1_fit, tau_1_fit = popt_1
    except Exception:
        Kp_1_fit, tau_1_fit = Kp_1, tau_1

    print(f"\n── Modelo 1ª ordem ──────────────────────")
    print(f"   Kp  = {Kp_1_fit:.6f}  (rad/PWM-unit)")
    print(f"   τ   = {tau_1_fit:.4f}  s")
    print(f"   G(s) = {Kp_1_fit:.4f} / ({tau_1_fit:.4f}·s + 1)")

    # ── Estimativa 2ª ordem ───────────────────────────────────
    # Chutes iniciais: wn baseado no tempo de subida, ζ = 0.7
    wn_0   = 2.2 / max(tau_1_fit, 0.01)
    zeta_0 = 0.7

    try:
        popt_2, _ = curve_fit(
            lambda t, Kp, wn, zeta: resposta_2a_ordem(t, Kp * dU_val, wn, zeta),
            t_seg, y_seg,
            p0=[Kp_1_fit, wn_0, zeta_0],
            maxfev=10000,
            bounds=([-np.inf, 1e-3, 1e-3], [np.inf, 200, 10])
        )
        Kp_2, wn_2, zeta_2 = popt_2
        ajuste_2a_ok = True
    except Exception as e:
        print(f"[AVISO] Ajuste 2ª ordem não convergiu: {e}")
        Kp_2, wn_2, zeta_2 = Kp_1_fit, wn_0, zeta_0
        ajuste_2a_ok = False

    print(f"\n── Modelo 2ª ordem ──────────────────────")
    print(f"   Kp   = {Kp_2:.6f}  (rad/PWM-unit)")
    print(f"   ωn   = {wn_2:.4f}  rad/s")
    print(f"   ζ    = {zeta_2:.4f}")
    print(f"   G(s) = {Kp_2:.4f}·{wn_2:.4f}² / (s²"
          f" + 2·{zeta_2:.4f}·{wn_2:.4f}·s + {wn_2:.4f}²)")

    # ── Curvas ajustadas ──────────────────────────────────────
    t_ajuste = np.linspace(0, t_seg[-1], 500)
    y_1a     = resposta_1a_ordem(t_ajuste, Kp_1_fit * dU_val, tau_1_fit)
    y_2a     = resposta_2a_ordem(t_ajuste, Kp_2 * dU_val, wn_2, zeta_2) \
               if ajuste_2a_ok else np.zeros_like(t_ajuste)

    # ── Figura ────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Pêndulo de Furuta — Ensaio em Malha Aberta\n"
                 "Estimativa da Função de Transferência",
                 fontsize=13, fontweight='bold')

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ── Subplot 1: U(t) ao longo de todo o ensaio ─────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.step(df['t_s'], df['pwm'], where='post', color='royalblue', lw=1.5,
             label='PWM aplicado U(t)')
    ax1.axhline(U_op, color='gray', ls='--', lw=1, label=f'U_op = {U_op}')
    ax1.set_ylabel('PWM (0–255)')
    ax1.set_xlabel('Tempo (s)')
    ax1.set_title('Entrada U(t) — sequência de degraus')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ── Subplot 2: Y(t) ao longo de todo o ensaio ─────────────
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(df['t_s'], df['alpha_rad'], color='darkorange', lw=1.2,
             label='α medido (rad)')
    ax2.axhline(Y_op, color='gray', ls='--', lw=1, label=f'Y_op ≈ {Y_op:.4f} rad')
    ax2.set_ylabel('Ângulo α (rad)')
    ax2.set_xlabel('Tempo (s)')
    ax2.set_title('Saída Y(t) — ângulo do pêndulo')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ── Subplot 3: Ajuste 1ª ordem ────────────────────────────
    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(t_seg, y_seg, 'o', color='darkorange', ms=3, alpha=0.6,
             label='Dados experimentais (δY)')
    ax3.plot(t_ajuste, y_1a, '-', color='blue', lw=2,
             label=f'1ª ordem: Kp={Kp_1_fit:.4f}, τ={tau_1_fit:.3f}s')
    ax3.set_title('Desvio δY — Modelo 1ª ordem')
    ax3.set_xlabel('Tempo (s)')
    ax3.set_ylabel('δα (rad)')
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.3)

    # ── Subplot 4: Ajuste 2ª ordem ────────────────────────────
    ax4 = fig.add_subplot(gs[2, 1])
    ax4.plot(t_seg, y_seg, 'o', color='darkorange', ms=3, alpha=0.6,
             label='Dados experimentais (δY)')
    if ajuste_2a_ok:
        ax4.plot(t_ajuste, y_2a, '-', color='green', lw=2,
                 label=f'2ª ordem: Kp={Kp_2:.4f}, ωn={wn_2:.3f}, ζ={zeta_2:.3f}')
    ax4.set_title('Desvio δY — Modelo 2ª ordem')
    ax4.set_xlabel('Tempo (s)')
    ax4.set_ylabel('δα (rad)')
    ax4.legend(fontsize=7)
    ax4.grid(True, alpha=0.3)

    # ── Salvar figura ─────────────────────────────────────────
    base      = Path(caminho_csv).stem
    nome_png  = Path(caminho_csv).parent / f"{base}_estimativa_tf.png"
    fig.savefig(nome_png, dpi=150, bbox_inches='tight')
    print(f"\n[OK] Gráfico salvo em: {nome_png}")

    plt.show()

    # ── Resumo final ──────────────────────────────────────────
    print("\n" + "="*55)
    print("  RESUMO — PARÂMETROS ESTIMADOS")
    print("="*55)
    print(f"  Ponto de operação:  U_op = {U_op}   Y_op = {Y_op:.6f} rad")
    print(f"\n  Modelo 1ª ordem:")
    print(f"    G(s) = {Kp_1_fit:.5f} / ({tau_1_fit:.5f}s + 1)")
    if ajuste_2a_ok:
        wn2 = wn_2**2
        print(f"\n  Modelo 2ª ordem:")
        print(f"    G(s) = {Kp_2:.5f}·{wn2:.4f} / "
              f"(s² + {2*zeta_2*wn_2:.4f}s + {wn2:.4f})")
    print("="*55)


# ── Entry point ───────────────────────────────────────────────

if __name__ == '__main__':
    if len(sys.argv) > 1:
        arquivo = sys.argv[1]
    else:
        # Tenta abrir seletor de arquivo (ambientes com GUI)
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk(); root.withdraw()
            arquivo = filedialog.askopenfilename(
                title="Selecione o CSV de dados",
                filetypes=[("CSV", "*.csv"), ("Todos", "*.*")]
            )
            if not arquivo:
                print("[ERRO] Nenhum arquivo selecionado.")
                sys.exit(1)
        except Exception:
            print("Uso: python estimar_tf.py <arquivo.csv>")
            sys.exit(1)

    if not os.path.isfile(arquivo):
        print(f"[ERRO] Arquivo não encontrado: {arquivo}")
        sys.exit(1)

    analisar(arquivo)
