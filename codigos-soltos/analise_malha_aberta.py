"""
analise_malha_aberta.py
─────────────────────────────────────────────────────────────
Análise dos dados de ensaio em malha aberta do Pêndulo de
Furuta (posição inferior).

Realiza:
  1. Leitura e pré-processamento do CSV
  2. Plotagem dos dados brutos (Y e U vs. tempo)
  3. Estimativa da função de transferência G(s) por ajuste
     - Tenta modelo de 1ª ordem:  G(s) = Kp / (tau*s + 1)
     - Tenta modelo de 2ª ordem:  G(s) = Kp*wn² / (s²+2ζwn·s+wn²)
  4. Plotagem comparativa: dados experimentais vs. modelo estimado

Uso:
    python analise_malha_aberta.py [arquivo_csv]

Dependências:
    pip install numpy matplotlib scipy pandas
─────────────────────────────────────────────────────────────
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.optimize import curve_fit, minimize
from scipy.signal import lsim, lti

# ── Arquivo de entrada ────────────────────────────────────────
arquivo = sys.argv[1] if len(sys.argv) > 1 else "dados_ensaio.csv"

print(f"[INFO] Lendo '{arquivo}'...")

try:
    df = pd.read_csv(arquivo, comment="#")
except FileNotFoundError:
    # Se não existe arquivo real, gera dados simulados para demonstração
    print("[AVISO] Arquivo não encontrado. Gerando dados simulados para demonstração.")

    np.random.seed(42)
    Ts   = 0.02
    t    = np.arange(0, 15, Ts)
    n    = len(t)

    # Sequência de PWM simulada (igual ao ensaio)
    U_OP = 127
    U_D  = 64
    T_P  = 3.0   # s por passo
    pwm_seq = np.zeros(n)
    for i, ti in enumerate(t):
        p = int(ti / T_P)
        if   p == 0: pwm_seq[i] = U_OP
        elif p == 1: pwm_seq[i] = U_OP + U_D
        elif p == 2: pwm_seq[i] = U_OP
        elif p == 3: pwm_seq[i] = U_OP - U_D
        else:        pwm_seq[i] = U_OP

    # Simula resposta de 2ª ordem subamortecida
    Kp  = 0.015   # rad/PWM
    wn  = 3.0     # rad/s
    zsi = 0.5
    sys_sim = lti([Kp * wn**2], [1, 2*zsi*wn, wn**2])

    U_delta = pwm_seq - U_OP
    _, alpha_ideal, _ = lsim(sys_sim, U_delta, t)
    alpha = alpha_ideal + np.random.normal(0, 0.002, n)  # ruído

    df = pd.DataFrame({
        "tempo_ms":        (t * 1000).astype(int),
        "passo":           [int(ti/T_P) for ti in t],
        "pwm":             pwm_seq.astype(int),
        "alpha_rad":       alpha,
        "alphaDot_rad_s":  np.gradient(alpha, Ts),
    })
    print("[INFO] Dados simulados gerados (Kp=0.015, wn=3.0 rad/s, ζ=0.5).")

# ── Pré-processamento ─────────────────────────────────────────
df.columns = df.columns.str.strip()
t_s     = df["tempo_ms"].values / 1000.0
t_s     = t_s - t_s[0]                   # começa em zero
alpha   = df["alpha_rad"].values
pwm     = df["pwm"].values.astype(float)

# Ponto de operação: mediana do passo 0
mask_op = df["passo"] == 0
Y_op    = np.median(alpha[mask_op]) if mask_op.any() else 0.0
U_op    = np.median(pwm[mask_op])   if mask_op.any() else 127.0

# Variáveis desviacionais (delta)
dY = alpha - Y_op
dU = pwm   - U_op

Ts = float(np.median(np.diff(t_s)))

print(f"[INFO] Amostras: {len(t_s)}")
print(f"[INFO] Duração : {t_s[-1]:.1f} s")
print(f"[INFO] Ts      : {Ts*1000:.1f} ms")
print(f"[INFO] Y_op    : {Y_op:.6f} rad  ({np.degrees(Y_op):.3f} °)")
print(f"[INFO] U_op    : {U_op:.1f} PWM")

# ── Estimativa da função de transferência ─────────────────────
# Estratégia: para cada degrau positivo (passo 1), isola a
# resposta e estima os parâmetros por minimização de erro RMS.

# Isolando o degrau positivo (passo 1 → Y sobe)
if 1 in df["passo"].values:
    idx_d = np.where(df["passo"].values == 1)[0]
    # inclui alguns pontos antes (estado inicial) e todo o passo seguinte
    i0 = max(0, idx_d[0] - 5)
    i1 = min(len(t_s)-1, idx_d[-1] + int(1.0/Ts))
    t_fit  = t_s[i0:i1] - t_s[i0]
    dU_fit = dU[i0:i1]
    dY_fit = dY[i0:i1]
else:
    t_fit  = t_s
    dU_fit = dU
    dY_fit = dY

def simular_1ordem(t, Kp, tau):
    """Simula resposta de G(s) = Kp/(tau*s+1) a entrada dU_fit."""
    if tau <= 0:
        return np.full_like(t, 1e9)
    sys_ = lti([Kp], [tau, 1])
    _, y, _ = lsim(sys_, dU_fit, t)
    return y

def simular_2ordem(t, Kp, wn, zsi):
    """Simula resposta de G(s) = Kp*wn²/(s²+2ζwn·s+wn²) a entrada dU_fit."""
    if wn <= 0 or zsi <= 0:
        return np.full_like(t, 1e9)
    sys_ = lti([Kp * wn**2], [1, 2*zsi*wn, wn**2])
    _, y, _ = lsim(sys_, dU_fit, t)
    return y

def rms(y_est, y_med):
    return np.sqrt(np.mean((y_est - y_med)**2))

# ── Ajuste 1ª ordem ───────────────────────────────────────────
print("\n[INFO] Ajustando modelo de 1ª ordem...")
try:
    p0_1 = [0.01, 1.0]
    popt1, _ = curve_fit(simular_1ordem, t_fit, dY_fit,
                         p0=p0_1, maxfev=5000,
                         bounds=([-10, 0.01], [10, 100]))
    Kp1, tau1 = popt1
    rms1 = rms(simular_1ordem(t_fit, *popt1), dY_fit)
    print(f"  Kp  = {Kp1:.6f}  rad/PWM")
    print(f"  tau = {tau1:.4f}  s")
    print(f"  RMS = {rms1:.6f} rad")
except Exception as e:
    print(f"  [AVISO] Ajuste 1ª ordem falhou: {e}")
    Kp1, tau1, rms1 = 0.01, 1.0, 1e9

# ── Ajuste 2ª ordem ───────────────────────────────────────────
print("\n[INFO] Ajustando modelo de 2ª ordem...")
try:
    p0_2 = [0.01, 3.0, 0.5]
    popt2, _ = curve_fit(simular_2ordem, t_fit, dY_fit,
                         p0=p0_2, maxfev=5000,
                         bounds=([-10, 0.1, 0.01], [10, 100, 5.0]))
    Kp2, wn2, zsi2 = popt2
    rms2 = rms(simular_2ordem(t_fit, *popt2), dY_fit)
    print(f"  Kp  = {Kp2:.6f}  rad/PWM")
    print(f"  wn  = {wn2:.4f}  rad/s")
    print(f"  ζ   = {zsi2:.4f}")
    print(f"  RMS = {rms2:.6f} rad")
except Exception as e:
    print(f"  [AVISO] Ajuste 2ª ordem falhou: {e}")
    Kp2, wn2, zsi2, rms2 = 0.01, 3.0, 0.5, 1e9

# Escolhe melhor modelo
melhor = "1ª ordem" if rms1 <= rms2 else "2ª ordem"
print(f"\n[INFO] Melhor ajuste: {melhor}  (RMS1={rms1:.6f}, RMS2={rms2:.6f})")

# Simula resposta do modelo escolhido para TODO o ensaio
if melhor == "1ª ordem":
    sys_final = lti([Kp1], [tau1, 1])
    label_tf  = f"G(s) = {Kp1:.4f} / ({tau1:.3f}·s + 1)"
else:
    sys_final = lti([Kp2 * wn2**2], [1, 2*zsi2*wn2, wn2**2])
    label_tf  = (f"G(s) = {Kp2:.4f}·{wn2:.3f}² / "
                 f"(s² + 2·{zsi2:.3f}·{wn2:.3f}·s + {wn2:.3f}²)")

_, dY_modelo, _ = lsim(sys_final, dU, t_s)
Y_modelo = dY_modelo + Y_op

# ── Plotagem ──────────────────────────────────────────────────
plt.style.use("seaborn-v0_8-whitegrid")
fig = plt.figure(figsize=(13, 10))
fig.suptitle("Pêndulo de Furuta — Ensaio em Malha Aberta\n"
             "UFMT/FAENG — Introdução a Sistemas de Controle 2026/1",
             fontsize=13, fontweight="bold")

gs = gridspec.GridSpec(3, 1, hspace=0.45)

# ── Subplot 1: Entrada U ─────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
ax1.step(t_s, pwm, where="post", color="#2c7bb6", linewidth=1.8, label="U (PWM)")
ax1.axhline(U_op, color="gray", linestyle="--", linewidth=1.0, label=f"U_op = {U_op:.0f}")
ax1.set_ylabel("PWM (0–255)", fontsize=10)
ax1.set_title("Entrada manipulada U(t)", fontsize=10)
ax1.legend(fontsize=9)
ax1.set_xlim([t_s[0], t_s[-1]])

# Marcadores de passo
cores_passo = ["#aec7e8", "#ffbb78", "#aec7e8", "#ff9896", "#aec7e8"]
for p in range(5):
    mask_p = df["passo"].values == p
    if mask_p.any():
        t_ini = t_s[np.where(mask_p)[0][0]]
        t_fim = t_s[np.where(mask_p)[0][-1]]
        ax1.axvspan(t_ini, t_fim, alpha=0.15, color=cores_passo[p],
                    label=f"passo {p}" if p in [1, 3] else None)

# ── Subplot 2: Saída Y (dados) ────────────────────────────────
ax2 = fig.add_subplot(gs[1])
ax2.plot(t_s, np.degrees(alpha), color="#d62728", linewidth=1.2,
         label="Y experimental (α)")
ax2.plot(t_s, np.degrees(Y_modelo), color="#1f77b4", linewidth=1.8,
         linestyle="--", label=f"Modelo: {label_tf}")
ax2.axhline(np.degrees(Y_op), color="gray", linestyle=":", linewidth=1.0,
            label=f"Y_op = {np.degrees(Y_op):.2f} °")
ax2.set_ylabel("Ângulo α (graus)", fontsize=10)
ax2.set_title("Saída controlada Y(t) — experimental vs. modelo", fontsize=10)
ax2.legend(fontsize=8)
ax2.set_xlim([t_s[0], t_s[-1]])

# ── Subplot 3: Erro de modelagem ─────────────────────────────
ax3 = fig.add_subplot(gs[2])
erro_deg = np.degrees(alpha) - np.degrees(Y_modelo)
ax3.plot(t_s, erro_deg, color="#9467bd", linewidth=1.0)
ax3.axhline(0, color="black", linewidth=0.8)
ax3.fill_between(t_s, erro_deg, 0, alpha=0.2, color="#9467bd")
ax3.set_ylabel("Erro (graus)", fontsize=10)
ax3.set_xlabel("Tempo (s)", fontsize=10)
ax3.set_title("Erro: Y_experimental − Y_modelo", fontsize=10)
ax3.set_xlim([t_s[0], t_s[-1]])

plt.savefig("resultado_malha_aberta.png", dpi=150, bbox_inches="tight")
print("\n[INFO] Gráfico salvo em 'resultado_malha_aberta.png'")
plt.show()

# ── Resumo final ──────────────────────────────────────────────
print("\n" + "="*55)
print("  RESUMO DA MODELAGEM")
print("="*55)
print(f"  Ponto de operação:")
print(f"    Y_op = {np.degrees(Y_op):.4f} ° = {Y_op:.6f} rad")
print(f"    U_op = {U_op:.1f} PWM")
print()
print(f"  Modelo de 1ª ordem  G(s) = Kp / (τs + 1)")
print(f"    Kp  = {Kp1:.6f}  rad/PWM")
print(f"    τ   = {tau1:.4f}  s")
print(f"    RMS = {rms1:.6f} rad")
print()
print(f"  Modelo de 2ª ordem  G(s) = Kp·ωn² / (s² + 2ζωn·s + ωn²)")
print(f"    Kp  = {Kp2:.6f}  rad/PWM")
print(f"    ωn  = {wn2:.4f}  rad/s")
print(f"    ζ   = {zsi2:.4f}")
print(f"    RMS = {rms2:.6f} rad")
print()
print(f"  ► Melhor modelo: {melhor}")
print("="*55)
