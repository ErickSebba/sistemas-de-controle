# -*- coding: utf-8 -*-
"""Sintonia robusta do PID 2-DOF (c=0, convencao do curso) - planta Entrega 2."""
import numpy as np, control as ct, warnings; warnings.filterwarnings("ignore")
from scipy.optimize import minimize
trapz = np.trapezoid if hasattr(np,"trapezoid") else np.trapz

Kp,zeta,wn=2.8968e-03,0.0743,9.1069
P=ct.tf([Kp*wn**2],[1.0,2*zeta*wn,wn**2]); s=ct.tf('s'); gamma=1/8.
r0=0.10; umax=110.0
def Cfull(Kc,Ti,Td):
    Tf=gamma*Td; return Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1))
def Hr2(Kc,Ti,Td,b,c):
    Tf=gamma*Td
    Cr=Kc*b+Kc/(Ti*s)+Kc*c*(Td*s/(Tf*s+1)); C=Cfull(Kc,Ti,Td)
    return ct.minreal(Cr*P/(1+C*P),verbose=False), C
t=np.linspace(0,4,2000)
def metrics(Kc,Ti,Td,b=1.0,c=0.0):
    H,C=Hr2(Kc,Ti,Td,b,c)
    if not np.all(np.real(ct.poles(H))<0): return None
    _,y=ct.step_response(H,t); Mp=max(0,np.max(y)-1)*100
    itae=trapz(t*np.abs(1-y),t)
    Hu=ct.minreal((Kc*b+Kc/(Ti*s))*P/(1+C*P)*0+ct.feedback(C,P),verbose=False) # esforco (aprox 1DOF)
    _,u=ct.step_response(ct.feedback(C,P)*r0,t); upk=np.max(np.abs(u))
    try: _,pm,_,_=ct.margin(C*P)
    except Exception: pm=0
    return dict(Mp=Mp,itae=itae,upk=upk,pm=pm,H=H,C=C)

def cost(x):
    Kc,Ti,Td=x
    if Kc<=0 or Ti<=0 or Td<=0: return 1e9
    m=metrics(Kc,Ti,Td,1.0,0.0)
    if m is None: return 1e9
    return (m['itae'] + 40*max(0,m['Mp']-12) + 8*max(0,60-m['pm'])
            + 0.5*max(0,m['upk']-umax))

best=(1e9,None)
for Kc in np.logspace(0.5,3.0,22):
    for Ti in np.logspace(-1.7,0.7,16):
        for Td in np.linspace(0.03,0.30,12):
            fc=cost([Kc,Ti,Td])
            if fc<best[0]: best=(fc,[float(Kc),float(Ti),float(Td)])
r=minimize(cost,best[1],method='Nelder-Mead',options={'xatol':1e-4,'fatol':1e-3,'maxiter':6000})
Kc,Ti,Td=r.x; Tf=gamma*Td
m=metrics(Kc,Ti,Td,1.0,0.0)
print("=== SINTONIA ROBUSTA (c=0, MF>=60) ===")
print(f"  Kc={Kc:.2f}  Ti={Ti:.4f}s  Td={Td:.4f}s  Tf={Tf:.5f}s  Ki={Kc/Ti:.1f}")
print(f"  MF={m['pm']:.1f} deg | Mp(b=1,c=0)={m['Mp']:.1f}% | pico u(r=0.1)={m['upk']:.0f} PWM")
S=ct.step_info(m['H']); print(f"  Tr={S['RiseTime']:.3f}s  Ts={S['SettlingTime']:.3f}s")

print("\n  Varredura de b (c=0) p/ escolher o peso:")
for b in [0,0.3,0.5,0.7,1.0]:
    mb=metrics(Kc,Ti,Td,b,0.0); Sb=ct.step_info(mb['H'])
    print(f"    b={b}: Mp={mb['Mp']:.1f}%  Tr={Sb['RiseTime']:.3f}s  Ts={Sb['SettlingTime']:.3f}s")

b=0.5
Tac=ct.step_info(Hr2(Kc,Ti,Td,b,0.0)[0])['SettlingTime']
print(f"\n=== Ts (Lab 11/12) ===  Tac(2-DOF b={b})={Tac:.3f}s")
print(f"  Regra Tac/20..30: {Tac/30*1000:.0f}..{Tac/20*1000:.0f} ms | Regra 2Tf: {2*Tf*1000:.0f} ms | HW: 20 ms")
for Ts in [0.02]:
    zD=(2*Tf-Ts)/(2*Tf+Ts); print(f"  Ts={Ts*1000:.0f}ms: z_D={zD:+.3f} {'OK' if zD>0 else 'RINGING'}")

np.savez('/home/claude/projeto_final/design.npz',
         num_P=[Kp*wn**2],den_P=[1.0,2*zeta*wn,wn**2],
         Kc=Kc,Ti=Ti,Td=Td,Tf=Tf,gamma=gamma,b=b,c=0.0,r0=r0,umax=umax,Ts=0.02)
print("\nOK salvo.")
