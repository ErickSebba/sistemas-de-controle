# -*- coding: utf-8 -*-
"""
FASE 0 - Calibracao do modelo nao linear (2 GDL, eqs. do TCC) contra o G(s) da Entrega 2.

Correcoes aplicadas ao TCC:
  * Armadilha 1: convencao angular. No TCC alpha=0 e' o pendulo PARA CIMA.
    Nossa posicao (inferior) e' alpha_TCC = pi. Trabalhamos com delta = alpha_TCC - pi.
    Na linearizacao em delta=0, a gravidade e' RESTAURADORA (+Kg*delta).
  * Armadilha 2: Ip da Tabela 2 (3,84e-4) e' a inercia no PIVO (= I_cm + mp*lp^2),
    nao no CM. Portanto o termo do TCC (mp*lp^2 + Ip) conta mp*lp^2 DUAS VEZES.
    Usamos J = inercia do pendulo no pivo, direto (e o calibramos).
  * Armadilha 3: beta_theta e beta_alpha do TCC sao SIMBOLICOS (p.50), nao medidos.
    Calibramos ambos.
  * Bloqueio: k_tau [N.m/PWM] nao esta no TCC. Calibramos.

Modelo linearizado (delta pequeno, em torno da posicao inferior):
    [a  c][theta_dd]   [b_th  0 ][theta_d]   [0   0 ][theta]   [tau]
    [c  J][delta_dd] + [0   b_al][delta_d] + [0  Kg ][delta] = [ 0 ]
com  a = Ib + mp*Lb^2 ;  c = mp*Lb*lp ;  Kg = mp*g*lp ;  J = inercia do pendulo no pivo.

    Delta(s)/T(s) = -c*s / [ (a*J - c^2)s^3 + (a*b_al + b_th*J)s^2
                             + (a*Kg + b_th*b_al)s + b_th*Kg ]
  -> ZERO NA ORIGEM  ->  GANHO DC = 0  (achado estrutural; ver relatorio)
"""
import os
import numpy as np, control as ct, warnings; warnings.filterwarnings("ignore")
from scipy.optimize import least_squares
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIGS_DIR = os.path.join(BASE_DIR, "figs")
os.makedirs(FIGS_DIR, exist_ok=True)
plt.rcParams.update({"font.size":10.5,"axes.grid":True,"grid.alpha":.3,
                     "figure.dpi":150,"savefig.bbox":"tight"})
AZUL,VERM,VERDE,CINZA="#1f4e79","#d62728","#2e7d32","#666"

# ---------------- constantes fisicas (TCC, Tabela 2) ----------------
g   = 9.81
mp  = 45e-3        # massa do pendulo (kg)
Lb  = 120e-3       # comprimento do braco (m)
lp  = 80e-3        # distancia do pivo ao CM do pendulo (m)
Ib  = 0.0015       # inercia do braco (kg.m^2)
Ip_tab = 384e-6    # Tabela 2 -> inercia do pendulo NO PIVO (ver Armadilha 2)

a  = Ib + mp*Lb**2        # inercia efetiva do braco
c  = mp*Lb*lp             # acoplamento inercial braco<->pendulo
Kg = mp*g*lp              # termo gravitacional restaurador (posicao INFERIOR)

# ---------------- alvo: G(s) da Entrega 2 (experimental) ----------------
Kp_dc, zeta_id, wn_id = 2.8968e-3, 0.0743, 9.1069
G_exp = ct.tf([Kp_dc*wn_id**2],[1.0, 2*zeta_id*wn_id, wn_id**2])   # PWM -> alpha (rad)

print("="*74)
print("FASE 0 - CALIBRACAO DO MODELO NAO LINEAR (2 GDL) DO TCC")
print("="*74)
print(f"  a  = Ib + mp*Lb^2 = {a:.6e} kg.m^2")
print(f"  c  = mp*Lb*lp     = {c:.6e} kg.m^2   (acoplamento)")
print(f"  Kg = mp*g*lp      = {Kg:.6e} N.m/rad")
print(f"  Ip (Tabela 2)     = {Ip_tab:.6e} kg.m^2  -> inercia NO PIVO")
print(f"     I_cm(barra)+mp*lp^2 = {mp*(2*lp)**2/12:.3e} + {mp*lp**2:.3e} = "
      f"{mp*(2*lp)**2/12 + mp*lp**2:.3e}  (confere)\n")

def G_2gdl(J, b_al, b_th, k_tau):
    """PWM -> delta (rad). Modelo 2 GDL linearizado + conversao k_tau (N.m/PWM)."""
    num = [-c*k_tau, 0.0]                       # -c*s*k_tau
    den = [a*J - c**2,
           a*b_al + b_th*J,
           a*Kg + b_th*b_al,
           b_th*Kg]
    return ct.tf(num, den)

# ---------------- funcao de custo: casar a resposta em frequencia ----------------
# Banda relevante para o controle (em torno da ressonancia). NAO inclui DC, pois os
# dois modelos sao estruturalmente diferentes em DC (ver achado estrutural).
w = np.logspace(np.log10(1.0), np.log10(100.0), 300)
mag_exp, ph_exp, _ = ct.frequency_response(G_exp, w)
mag_exp = np.squeeze(mag_exp)

def residuo(x):
    J, b_al, b_th, k_tau = x
    if J <= 0 or b_al <= 0 or b_th <= 0 or (a*J - c**2) <= 0:
        return 1e3*np.ones_like(w)
    Gm = G_2gdl(J, b_al, b_th, k_tau)
    mag, _, _ = ct.frequency_response(Gm, w)
    mag = np.squeeze(mag)
    # residuo em dB (pondera igualmente toda a decada em torno da ressonancia)
    return 20*np.log10(np.maximum(mag,1e-12)) - 20*np.log10(np.maximum(mag_exp,1e-12))

# chute inicial informado pela fisica
J0    = Kg/wn_id**2 + c**2/a       # J que reproduz wn com acoplamento
bal0  = 2*zeta_id*wn_id*J0         # atrito do pendulo compativel com zeta
bth0  = 0.01                       # valor "simbolico" do TCC
ktau0 = 1e-4
x0 = [J0, bal0, bth0, ktau0]

sol = least_squares(residuo, x0, method='trf',
                    bounds=([1e-5, 1e-6, 1e-5, 1e-8],
                            [5e-3, 1e-1, 1.0,  1e-1]),
                    xtol=1e-14, ftol=1e-14, max_nfev=8000)
J, b_al, b_th, k_tau = sol.x
G_cal = G_2gdl(J, b_al, b_th, k_tau)

# ---------------- diagnostico: wn e zeta efetivos do modelo calibrado ----------------
pol = ct.poles(G_cal)
pc  = [p for p in pol if abs(np.imag(p)) > 1e-6]
if pc:
    p1 = pc[0]
    wn_cal   = float(abs(p1))
    zeta_cal = float(-np.real(p1)/abs(p1))
else:
    wn_cal = zeta_cal = float('nan')

print("-"*74)
print("RESULTADO DA CALIBRACAO   (TCC 'simbolico'  ->  calibrado nos dados)")
print("-"*74)
print(f"{'Parametro':<34}{'TCC / teorico':>16}{'Calibrado':>16}   {'obs'}")
print(f"{'J  - inercia pendulo no pivo':<34}{Ip_tab:>16.4e}{J:>16.4e}   {'+%.0f%%'%(100*(J/Ip_tab-1))}")
print(f"{'b_alpha - atrito do pendulo':<34}{0.001:>16.4e}{b_al:>16.4e}   simbolico no TCC")
print(f"{'b_theta - atrito do braco':<34}{0.01:>16.4e}{b_th:>16.4e}   MAL CONDICIONADO")
print(f"{'k_tau [N.m/PWM]':<34}{'(ausente)':>16}{k_tau:>16.4e}   nao existe no TCC")
print()
print(f"  Modelo calibrado:  wn = {wn_cal:.4f} rad/s  (alvo {wn_id:.4f})")
print(f"                     zeta = {zeta_cal:.4f}      (alvo {zeta_id:.4f})")
print(f"  Erro de ajuste (RMS na banda 1-100 rad/s): {np.sqrt(np.mean(sol.fun**2)):.3f} dB")
print()
print(f"  Acoplamento inercial c^2/(a*J) = {100*c**2/(a*J):.1f}%  (NAO desprezivel)")
print(f"  Ganho DC do 2 GDL = {float(ct.dcgain(G_cal)):.3e}   <-- ZERO (zero na origem)")
print(f"  Ganho DC do G(s)  = {Kp_dc:.4e}   <-- FINITO")
print("  -> os dois modelos NAO sao equivalentes em baixa frequencia. Ver relatorio.")

# ---------------- sensibilidade de b_theta (honestidade sobre a incerteza) --------
print("\n" + "-"*74)
print("SENSIBILIDADE DE b_theta (por que ele e' mal condicionado)")
print("-"*74)
base = np.sqrt(np.mean(residuo(sol.x)**2))
print(f"{'b_theta':>12}{'RMS (dB)':>12}   (com J, b_al, k_tau fixos no otimo)")
for f in [0.1, 0.5, 1.0, 2.0, 10.0]:
    r = np.sqrt(np.mean(residuo([J, b_al, b_th*f, k_tau])**2))
    print(f"{b_th*f:>12.4e}{r:>12.3f}   {'<- otimo' if abs(f-1)<1e-9 else ''}")
print("  b_theta pode variar 10x com efeito quase nulo na banda de controle:")
print("  ele domina o DC (onde os modelos ja divergem), nao a ressonancia.")

# ---------------- FIGURA: Bode G(s) x 2 GDL calibrado ----------------
mag_c, ph_c, _ = ct.frequency_response(G_cal, w)
mag_c = np.squeeze(mag_c)
fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6.0), sharex=True)
a1.semilogx(w, 20*np.log10(mag_exp), color=AZUL, lw=2.2, label='G(s) — Entrega 2 (experimental)')
a1.semilogx(w, 20*np.log10(mag_c),  '--', color=VERM, lw=1.8, label='2 GDL do TCC — calibrado')
a1.axvline(wn_id, color=CINZA, ls=':', lw=1)
a1.set_ylabel('|G| (dB)')
a1.set_title('Fase 0: modelo 2 GDL (TCC) calibrado contra o G(s) da Entrega 2')
a1.legend(fontsize=9, loc='lower left')
a2.semilogx(w, np.degrees(np.squeeze(ph_exp)), color=AZUL, lw=2.2)
a2.semilogx(w, np.degrees(np.squeeze(ph_c)), '--', color=VERM, lw=1.8)
a2.axvline(wn_id, color=CINZA, ls=':', lw=1)
a2.set_ylabel('fase (graus)'); a2.set_xlabel('ω (rad/s)')
a2.text(1.2, np.degrees(np.squeeze(ph_c))[0]+10,
        'em DC os modelos divergem\n(2 GDL tem zero na origem)',
        fontsize=8, color=VERM)
fig.savefig(os.path.join(FIGS_DIR, 'n1_calibracao.png')); plt.close(fig)

np.savez(os.path.join(BASE_DIR, 'params_2gdl.npz'),
         a=a, c=c, Kg=Kg, J=J, b_al=b_al, b_th=b_th, k_tau=k_tau,
         mp=mp, Lb=Lb, lp=lp, Ib=Ib, g=g)
print("\nOK -> figs/n1_calibracao.png  e  params_2gdl.npz")
