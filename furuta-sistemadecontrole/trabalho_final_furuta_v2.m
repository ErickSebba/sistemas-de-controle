%% ============================================================================
%  TRABALHO FINAL - Controle PID 2-DOF do Pendulo de Furuta (posicao INFERIOR)
%  Disciplina: Introducao aos Sistemas de Controle - 2026/1
%  Prof. Me. Daniel Miranda Cruz  |  UFMT/FAENG - Eng. de Computacao
%  Grupo: Daniel M. Zamarioli, Erick G. B. Sebba, Fabio C. Barbosa Jr.
%
%  Estruturado como os laboratorios (Labs 08 a 12). Roda SOZINHO (equacoes a
%  diferencas). CENARIO REGULATORIO: pendulo mantido em alpha = 0 graus.
%
%  Secao 1  - Processo (modelo Entrega 2) e ponto de operacao
%  Secao 2  - BATERIA DE SINTONIA (estilo Lab 09): varios metodos, inclusive os
%             INSTAVEIS, com tabela comparativa  ->  escolhemos a melhor
%  Secao 3  - Analise em malha fechada 2-DOF (Lab 10): Hr, Hu, Hd
%  Secao 4  - Discretizacao Tustin: C(z) e F(z)  (Lab 11 - Parte 1)
%  Secao 5  - Malha inteira num laco for: continuo x discreto (Lab 11 - Parte 2)
%  Secao 6  - 2-DOF discreto: onde o peso b atua?  (Lab 11 - Parte 3)
%  Secao 7  - Varredura de Ts: degradacao e instabilidade (Lab 11 - Parte 4)
%  Secao 8  - Zona morta do motor + compensacao inversa  (nao-linearidade real)
%  Secao 9  - Cenario REGULATORIO: rejeicao de perturbacao em alpha = 0
%
%  Forma ISA:  C(s) = Kc(1 + 1/(Ti s) + Td s/(gamma Td s + 1)),  gamma = 1/8
%  2-DOF (Lab 10): b pondera R no proporcional; c=0 (derivada so na medicao).
%% ============================================================================
clear all; close all; clc;

%% 1 - PROCESSO (Entrega 2) e ponto de operacao ------------------------------
Kp = 2.8968e-3;  zeta = 0.0743;  wn = 9.1069;   % PWM -> alpha (rad)
s = tf('s');
G = (Kp*wn^2)/(s^2 + 2*zeta*wn*s + wn^2);       % 2a ordem, ressonante, SEM atraso
gamma = 1/8;
U_op = 127;  Y_op = 0;                          % 50% PWM ; 0 graus (regulador)

fprintf('=== PROCESSO ===  G(s)=%.4f/(s^2+%.4f s+%.4f)  [Kp=%.2e zeta=%.3f wn=%.2f]\n\n',...
        Kp*wn^2, 2*zeta*wn, wn^2, Kp, zeta, wn);

%% 2 - BATERIA DE SINTONIA (estilo Lab 09) -----------------------------------
%  A planta e' 2a ordem RESSONANTE e SEM tempo morto. As tabelas empiricas FOPDT
%  (ZN/CHR/Cohen-Coon) pressupoem 1a ordem + atraso: forcar um ajuste FOPDT da um
%  Kc absurdo (satura na hora). Metodos sem derivativo NAO amortecem a ressonancia
%  (ficam instaveis). Testamos varios e comparamos.
%
%  FOPDT forcado (metodo da area): tau+theta = a1/a2 (~16 ms). Com um "atraso
%  aparente" theta~0.10 s -> ZN da Kc ~ 5800 (inutil).  Ver relatorio.

metodos = {  % nome                         Kc      Ti      Td
   'ZN via FOPDT (empirico)'        5799.5   0.200   0.050
   'PI puro (sem derivativo)'         80.0   0.120   0.000
   'PID agressivo (banda alta)'      320.0   0.030   0.120
   'PID robusto A (ADOTADO)'          57.31  0.1015  0.4777
   'PID rapido B'                    142.22  0.0452  0.3805
};
fprintf('=== BATERIA DE SINTONIA (Lab 09) ===\n');
fprintf('%-30s %8s %8s %8s %9s %6s %7s %8s\n','Metodo','Kc','Ti','Td','estavel','MF','Mp%','Ts(s)');
for i = 1:size(metodos,1)
    Kc=metodos{i,2}; Ti=metodos{i,3}; Td=metodos{i,4};
    if Td>0, Tf=gamma*Td; C=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
    else,    C=Kc*(1+1/(Ti*s)); end
    H=feedback(C*G,1); est = all(real(pole(H))<1e-6);
    [~,MF]=margin(C*G);
    if est, S=stepinfo(H); mp=sprintf('%.0f',S.Overshoot); ts=sprintf('%.2f',S.SettlingTime);
    else,   mp='INST'; ts='INST'; end
    fprintf('%-30s %8.1f %8.3f %8.3f %9d %6.0f %7s %8s\n',...
            metodos{i,1},Kc,Ti,Td,est,MF,mp,ts);
end
fprintf(['-> Escolhida a A (robusta): margem alta (MF~53), sem overshoot e\n'...
         '   esforco moderado. B fica como alternativa rapida.\n\n']);

%% 3 - SINTONIA ADOTADA e analise 2-DOF em malha fechada (Lab 10) -------------
Kc = 57.31;  Ti = 0.1015;  Td = 0.4777;  Tf = gamma*Td;
b  = 0.7;    c  = 0;       Ts = 0.02;

C  = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
F  = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);   % pre-filtro (carrega b)
Hr = F*feedback(C*G,1);    % R -> Y  (seguimento 2-DOF)
Hu = F*feedback(C,G);      % R -> U  (esforco)
Hd = feedback(G,C);        % D -> Y  (rejeicao; NAO depende de b,c)
[Gm,Pm,~,Wcp]=margin(C*G); Sr=stepinfo(feedback(C*G,1));
fprintf('=== SINTONIA ADOTADA ===  Kc=%.2f Ti=%.4f Td=%.4f b=%.1f c=%.0f Ts=%.3f\n',Kc,Ti,Td,b,c,Ts);
fprintf('  MF=%.1f deg | wc=%.2f rad/s | Mp=%.1f%% | Ts_ac=%.2f s\n\n',Pm,Wcp,Sr.Overshoot,Sr.SettlingTime);

%% 4 - DISCRETIZACAO TUSTIN (Lab 11 - Parte 1) -------------------------------
Cz=c2d(C,Ts,'tustin'); [numCz,denCz]=tfdata(Cz,'v'); numCz=numCz/denCz(1); denCz=denCz/denCz(1);
Fz=c2d(F,Ts,'tustin'); [numFz,denFz]=tfdata(Fz,'v'); numFz=numFz/denFz(1); denFz=denFz/denFz(1);
zD=(2*Tf-Ts)/(2*Tf+Ts);
fprintf('=== DISCRETIZACAO (Tustin, Ts=%.0f ms) ===\n',Ts*1000);
fprintf('  C(z): q0=%.4f q1=%.4f q2=%.4f | den 1 %.4f %.4f\n',numCz(1),numCz(2),numCz(3),denCz(2),denCz(3));
fprintf('  F(z): n0=%.5f n1=%.5f n2=%.5f | den 1 %.4f %.4f\n',numFz(1),numFz(2),numFz(3),denFz(2),denFz(3));
fprintf('  Ts: 2Tf=%.0f ms | Tac/30=%.0f ms | 2pi/wn/10=%.0f ms | zD=%+.3f (OK se >0)\n\n',...
        2*Tf*1000, Sr.SettlingTime/30*1000, 2*pi/wn/10*1000, zD);

%% 5 - MALHA INTEIRA NUM LACO FOR: continuo x discreto (Lab 11 - Parte 2) -----
ref = 10;  tsim = 6;
[t5,y5,~] = malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tsim,ref,0,inf,130,75,255,false); % sem DZ
tc=0:0.005:tsim; yc=rad2deg(deg2rad(ref)*step(Hr,tc));
figure('Name','Parte 2 - continuo x discreto');
plot(tc,yc,'LineWidth',2); hold on; stairs(t5,y5,'LineWidth',1.4);
yline(ref,'--'); title('Lab 11 - Parte 2: continuo x discreto');
ylabel('\alpha (graus)'); xlabel('t (s)'); legend('continuo','discreto (Tustin)','Location','SE'); grid on;

%% 6 - 2-DOF DISCRETO: onde o peso b atua? (Lab 11 - Parte 3) -----------------
%  Servo (degrau) em t=0 e perturbacao em t=3 s. b muda o SERVO, nao a regulacao.
[t6,yA,~]=malha_furuta(Ts,0.5,Kc,Ti,Td,Tf,gamma,7,10,40,3,130,75,255,true);
[~ ,yB,~]=malha_furuta(Ts,1.0,Kc,Ti,Td,Tf,gamma,7,10,40,3,130,75,255,true);
figure('Name','Parte 3 - 2-DOF');
stairs(t6,yA,'LineWidth',1.5); hold on; stairs(t6,yB,'LineWidth',1.5);
yline(10,'--'); xline(3,':'); title('Lab 11 - Parte 3: b=0,5 x b=1,0 (servo em t=0; perturbacao em t=3s)');
ylabel('\alpha (graus)'); xlabel('t (s)'); legend('b=0,5','b=1,0','Location','SE'); grid on;
iserv = t6<3;
fprintf('=== Parte 3 (2-DOF) ===  max|yA-yB| APOS a perturbacao = %.3f graus (regulacao nao ve b)\n\n',...
        max(abs(yA(~iserv)-yB(~iserv))));

%% 7 - VARREDURA DE Ts: degradacao -> instabilidade (Lab 11 - Parte 4) --------
%  Com a sintonia RAPIDA B (MF~40): ao crescer Ts, a margem se esgota e a malha
%  fica INSTAVEL (mesma licao do Lab, agora sem tempo morto - o culpado e' o Ts).
KcB=142.22; TiB=0.0452; TdB=0.3805; TfB=gamma*TdB; bB=0.5;
figure('Name','Parte 4 - varredura de Ts'); hold on; cores=lines(5); i=0;
for Tsi=[0.02 0.05 0.10 0.15 0.20]
    i=i+1;
    [t7,y7,inst]=malha_furuta(Tsi,bB,KcB,TiB,TdB,TfB,gamma,4,10,0,inf,130,75,255,false);
    tag=''; if inst, tag=' (INSTAVEL)'; end
    stairs(t7,y7,'LineWidth',1.4,'Color',cores(i,:),'DisplayName',sprintf('Ts=%.0f ms%s',Tsi*1000,tag));
    fprintf('Parte 4: Ts=%3.0f ms | zD=%+.3f | %s\n',Tsi*1000,(2*TfB-Tsi)/(2*TfB+Tsi),ternario(inst,'INSTAVEL','estavel'));
end
yline(10,'--','DisplayName','ref'); ylim([-10 45]);
title('Lab 11 - Parte 4: varredura de Ts (sintonia B)'); ylabel('\alpha (graus)'); xlabel('t (s)');
legend('Location','NE'); grid on; fprintf('\n');

%% 8 - ZONA MORTA do motor + compensacao inversa -----------------------------
%  Medida: sentido + parte em ~120-140 PWM; sentido - em ~70-80 PWM.
DZP=130; DZN=75; Umax=255;
[t8c,y8c,~]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,6,10,0,inf,DZP,DZN,Umax,true);   % COM comp
[t8s,y8s,~]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,6,10,0,inf,DZP,DZN,Umax,false);  % SEM comp
figure('Name','Zona morta');
stairs(t8s,y8s,'r','LineWidth',1.5); hold on; stairs(t8c,y8c,'b','LineWidth',1.7); yline(10,'--');
title('Zona morta assimetrica (+130/-75 PWM): sem x com compensacao');
ylabel('\alpha (graus)'); xlabel('t (s)'); legend('SEM compensacao','COM compensacao','Location','SE'); grid on;
fprintf('=== ZONA MORTA ===  compensacao inversa OBRIGATORIA (esforco ~ banda morta)\n\n');

%% 9 - CENARIO REGULATORIO: rejeicao de perturbacao em alpha = 0 -------------
%  Principal (0 graus). Para testar outro ponto, troque ref_deg (ex.: 5).
ref_deg = 0;  d_pwm = 40;  t_dist = 2.0;  tfim = 8;
[t9,y9,u9]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tfim,ref_deg,d_pwm,t_dist,DZP,DZN,Umax,true);
[~ ,y9o]  =malha_furuta_ol(Ts,tfim,d_pwm,t_dist,G);   % sem controle (referencia)
figure('Name','Regulatorio');
subplot(2,1,1);
plot(t9,y9o,'r','LineWidth',1.3); hold on; stairs(t9,y9,'b','LineWidth',1.8);
yline(0,'--'); xline(t_dist,':'); title('Cenario REGULATORIO: perturbacao 40 PWM em t=2 s, mantendo \alpha=0');
ylabel('\alpha (graus)'); legend('sem controle','com PID 2-DOF','Location','NE'); grid on;
subplot(2,1,2); stairs(t9,u9,'b','LineWidth',1.3); yline(DZP,':'); yline(-DZN,':');
ylabel('u (PWM)'); xlabel('t (s)'); grid on;
fprintf('=== REGULATORIO ===  pico=%.2f graus | alpha final=%.3f graus\n',max(abs(y9)),y9(end));

fprintf('\nFim. Sintonia na Secao 3; teste outro ponto de operacao na Secao 9.\n');

%% ======================= FUNCOES LOCAIS ====================================
function v = pega(x,k)
    if k>=1, v=x(k); else, v=0; end
end
function r = ternario(cond,a,b)
    if cond, r=a; else, r=b; end
end

function [t,y,u,inst] = malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tsim,ref_deg, ...
                          d_pwm,t_dist, DZP,DZN,Umax, comp)
% Malha fechada discreta do Furuta (2a ordem, SEM atraso) no padrao Lab 11:
%   R -> F(z) -> (soma +-) -> C(z) -> saturacao -> comp. deadzone -> DZ motor
%   -> (+ perturbacao) -> planta discreta (ZOH). c=0 (derivada so na medicao).
    s=tf('s');
    C=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
    F=tf([b*Ti*Tf, b*Ti+Tf, 1],[Ti*Td*(1+gamma), Ti+Tf, 1]);
    Cz=c2d(C,Ts,'tustin'); [nC,dC]=tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
    Fz=c2d(F,Ts,'tustin'); [nF,dF]=tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
    Kp=2.8968e-3; zeta=0.0743; wn=9.1069;
    Gd=c2d((Kp*wn^2)/(s^2+2*zeta*wn*s+wn^2),Ts,'zoh');
    [nb,na]=tfdata(Gd,'v'); nb=nb/na(1); na=na/na(1);
    b1=nb(2); b2=nb(3);                         % y(k)=-a1 y1 -a2 y2 + b1 u1 + b2 u2
    N=round(tsim/Ts); t=(0:N-1)'*Ts;
    r=deg2rad(ref_deg)*ones(N,1); dpert=d_pwm*(t>=t_dist);
    rf=zeros(N,1); e=zeros(N,1); v=zeros(N,1); ucmd=zeros(N,1); ueff=zeros(N,1); yr=zeros(N,1);
    inst=false;
    for k=1:N
        yr(k)=-na(2)*pega(yr,k-1)-na(3)*pega(yr,k-2) ...
              + b1*(pega(ueff,k-1)+pega(dpert,k-1)) + b2*(pega(ueff,k-2)+pega(dpert,k-2));
        if abs(yr(k))>deg2rad(90), inst=true; yr(k:end)=NaN; v(k:end)=NaN; break; end
        rf(k)=-dF(2)*pega(rf,k-1)-dF(3)*pega(rf,k-2)+nF(1)*r(k)+nF(2)*pega(r,k-1)+nF(3)*pega(r,k-2);
        e(k)=rf(k)-yr(k);
        v(k)=-dC(2)*pega(v,k-1)-dC(3)*pega(v,k-2)+nC(1)*e(k)+nC(2)*pega(e,k-1)+nC(3)*pega(e,k-2);
        v(k)=min(max(v(k),-Umax),Umax);
        if comp, ucmd(k)=inv_deadzone(v(k),DZP,DZN); else, ucmd(k)=v(k); end
        ucmd(k)=min(max(ucmd(k),-Umax),Umax);
        ueff(k)=deadzone(ucmd(k),DZP,DZN);
    end
    y=rad2deg(yr); u=v;
end

function [t,y] = malha_furuta_ol(Ts,tsim,d_pwm,t_dist,G)
% Sem controle: so a perturbacao entra na planta (referencia visual).
    Gd=c2d(G,Ts,'zoh'); [nb,na]=tfdata(Gd,'v'); nb=nb/na(1); na=na/na(1);
    N=round(tsim/Ts); t=(0:N-1)'*Ts; dpert=d_pwm*(t>=t_dist); y=zeros(N,1);
    for k=1:N
        y(k)=-na(2)*pega(y,k-1)-na(3)*pega(y,k-2)+nb(2)*pega(dpert,k-1)+nb(3)*pega(dpert,k-2);
    end
    y=rad2deg(y);
end

function ue = deadzone(u,DZP,DZN)
    if u>DZP, ue=u-DZP; elseif u<-DZN, ue=u+DZN; else, ue=0; end
end
function uc = inv_deadzone(v,DZP,DZN)
    if v>1e-9, uc=v+DZP; elseif v<-1e-9, uc=v-DZN; else, uc=0; end
end

function [u,est] = pid_discreto(r,y,est,par)
% PID 2-DOF discreto FORMA 3 TERMOS, pronto para virar firmware (Arduino).
% Chamado a cada Ts. P com peso b | I trapezoidal com clamping | D filtrada
% sobre -y (c=0). Saturacao + compensacao de deadzone DENTRO do controlador.
    e = r - y;  P = par.Kc*(par.b*r - y);  vD = -y;
    aD=(2*par.Tf-par.Ts)/(2*par.Tf+par.Ts);  bD=2*par.Kc*par.Td/(2*par.Tf+par.Ts);
    est.ud = aD*est.ud + bD*(vD-est.v_ant);
    I_cand = est.I + (par.Kc*par.Ts/(2*par.Ti))*(e+est.e_ant);
    u_ns = P + I_cand + est.ud;
    if u_ns>1e-9, u_cmd=u_ns+par.DZP; elseif u_ns<-1e-9, u_cmd=u_ns-par.DZN; else, u_cmd=0; end
    u = min(max(u_cmd,-par.Umax),par.Umax);
    if ~((u~=u_cmd) && (sign(e)==sign(u_ns))), est.I = I_cand; end
    est.e_ant=e; est.v_ant=vD;
end
