"""
Estimacao da funcao de transferencia do pendulo de Furuta (referencia para BAIXO).
Sistema de 2a ordem ESTAVEL:  G(s) = Kp*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

Estrategia (a mais robusta para estes dados):
  - DENOMINADOR (zeta, wn)  -> da RESPOSTA LIVRE ("soltando"): ajuste de senoide
    amortecida no trecho APOS a soltura. A fase de subida (levantar o pendulo
    com a mao) e ZERADA/descartada, pois nao e dinamica do sistema.
  - GANHO (Kp)              -> dos IMPULSOS/DEGRAUS: simula-se G(s) com a entrada
    PWM real (lsim) e otimiza-se Kp (e confirma-se zeta, wn).

Requisitos: numpy, pandas, scipy, matplotlib
"""
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import lsim
from scipy.optimize import curve_fit, least_squares

# ----------------------------------------------------------------------
def carregar(caminho):
    df = pd.read_csv(caminho)
    t = df["tempo_ms"].values / 1000.0
    return t - t[0], df["pwm"].values.astype(float), df["alpha_rad"].values

# ----------------------------------------------------------------------
# 1) RESPOSTA LIVRE -> zeta, wn  (descartando a subida)
# ----------------------------------------------------------------------
def detectar_soltura(t, a, lim_vel=8.0):
    """Acha o instante de soltura: fim do plato onde o pendulo e segurado.
    Heuristica: ponto de |alpha| maximo sustentado seguido de queda rapida."""
    da = np.gradient(a, t)
    # candidato: ultimo indice antes de a velocidade disparar (soltura)
    idx_pico = np.argmax(np.abs(a))                  # angulo segurado (extremo)
    # avanca ate a velocidade ficar grande (comecou a cair)
    j = idx_pico
    while j < len(a) - 1 and abs(da[j]) < lim_vel:
        j += 1
    return j

def ajustar_resposta_livre(t, a, t_soltura=None, amp_max=None):
    if t_soltura is None:
        j = detectar_soltura(t, a)
    else:
        j = int(np.searchsorted(t, t_soltura))
    tt = t[j:] - t[j];  aa = a[j:].copy()
    # opcional: limitar a regiao linear (pequena amplitude)
    if amp_max is not None:
        k = np.argmax(np.abs(aa) <= amp_max)         # 1o ponto dentro do limite
        tt, aa = tt[k:] - tt[k], aa[k:]
    # frequencia inicial via cruzamentos por zero
    ac = aa - aa.mean(); zc = np.where(np.diff(np.sign(ac)))[0]
    wd0 = np.pi / np.mean(np.diff(tt[zc])) if len(zc) > 2 else 9.0
    def modelo(t, A, zeta, wn, phi, C):
        wd = wn * np.sqrt(max(1 - zeta**2, 1e-9))
        return A * np.exp(-zeta * wn * t) * np.sin(wd * t + phi) + C
    p, _ = curve_fit(modelo, tt, aa,
                     p0=[(aa.max()-aa.min())/2, 0.07, wd0, 0, aa.mean()],
                     maxfev=200000,
                     bounds=([-5,0,1,-2*np.pi,-1],[5,0.95,60,2*np.pi,1]))
    A, zeta, wn, phi, C = p
    return zeta, wn, (tt, aa, modelo(tt, *p))

# ----------------------------------------------------------------------
# 2) IMPULSOS/DEGRAUS -> Kp (e confirma zeta, wn) via lsim
# ----------------------------------------------------------------------
def ajustar_entrada_saida(t, u, y, dt=0.02, p0=(3e-3, 0.06, 9.2)):
    tu = np.arange(0, t[-1], dt)
    uu = np.interp(tu, t, u); yy = np.interp(tu, t, y)
    def sim(p):
        Kp, zeta, wn = p
        _, yo, _ = lsim(([Kp*wn**2], [1, 2*zeta*wn, wn**2]), uu, tu)
        return yo
    sol = least_squares(lambda p: sim(p) - yy, p0,
                        bounds=([1e-6,1e-3,1],[1,0.95,60]), max_nfev=8000)
    yhat = sim(sol.x)
    R2 = 1 - np.sum((yhat-yy)**2)/np.sum((yy-yy.mean())**2)
    return sol.x, R2, (tu, yy, yhat)

# ----------------------------------------------------------------------
if __name__ == "__main__":
    # ---- denominador a partir das duas respostas livres ----
    res = []
    for f in ["dados_malha_aberta_soltando2.csv", "dados_malha_aberta_soltando.csv"]:
        t, u, a = carregar(f)
        zeta, wn, _ = ajustar_resposta_livre(t, a, amp_max=0.6)
        res.append((zeta, wn)); print(f"{f}: zeta={zeta:.4f}  wn={wn:.4f} rad/s")
    zeta = np.mean([r[0] for r in res]); wn = np.mean([r[1] for r in res])

    # ---- ganho a partir dos impulsos ----
    t, u, a = carregar("dados_malha_aberta_20260610_132412.csv")
    (Kp, z2, wn2), R2, ov = ajustar_entrada_saida(t, u, a, p0=(3e-3, zeta, wn))
    print(f"\nImpulsos: Kp={Kp:.4e}  (zeta={z2:.4f}, wn={wn2:.4f}, R2={R2:.3f})")

    num = Kp*wn**2; a1 = 2*zeta*wn; a0 = wn**2
    print("\n=== MODELO FINAL ===")
    print(f"  Kp={Kp:.4e}   zeta={zeta:.4f}   wn={wn:.4f} rad/s")
    print(f"  G(s) = {num:.5f} / (s^2 + {a1:.4f} s + {a0:.4f})")

    # ---- cereja do bolo: sobreposicao ----
    fig, ax = plt.subplots(2, 1, figsize=(12, 8))
    t, u, a = carregar("dados_malha_aberta_soltando2.csv")
    _, _, (tt, aa, fit) = ajustar_resposta_livre(t, a, amp_max=0.6)
    ax[0].plot(tt, aa, "b", label="Experimental"); ax[0].plot(tt, fit, "r--", label="Modelo")
    ax[0].set_title("Resposta livre"); ax[0].legend(); ax[0].grid(alpha=.3)
    tu, yy, yhat = ov
    ax[1].plot(tu, yy, "b", label="Experimental"); ax[1].plot(tu, yhat, "r--", label="G(s)")
    ax[1].set_xlim(33, 41); ax[1].set_title("Impulso"); ax[1].legend(); ax[1].grid(alpha=.3)
    plt.tight_layout(); plt.savefig("sobreposicao.png", dpi=110)
    print("\nFigura salva: sobreposicao.png")
