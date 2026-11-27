%% ============================================================================
%  TRABALHO FINAL - Controle PID 2-DOF do Pendulo de Furuta (posicao INFERIOR)
%  Disciplina: Introducao aos Sistemas de Controle - 2026/1
%  Prof. Me. Daniel Miranda Cruz  |  UFMT/FAENG - Eng. de Computacao
%  Grupo: Daniel M. Zamarioli, Erick G. B. Sebba, Fabio C. Barbosa Jr.
%
%  Estruturado como os laboratorios (Labs 08 a 12). Roda SOZINHO (equacoes a
%  diferencas). CENARIO REGULATORIO: pendulo mantido em alpha = 0 graus.
%
%  Secao 1 - Processo (Entrega 2) e ponto de operacao
%  Secao 2 - BATERIA DE SINTONIA (Lab 09): metodos, inclusive os INSTAVEIS
%  Secao 3 - Sintonia adotada + analise 2-DOF em malha fechada (Lab 10)
%  Secao 4 - Discretizacao Tustin: C(z), F(z)             (Lab 11 - Parte 1)
%  Secao 5 - Laco for: continuo x discreto (LINEAR)       (Lab 11 - Parte 2)
%  Secao 6 - 2-DOF discreto: onde o peso b atua? (LINEAR) (Lab 11 - Parte 3)
%  Secao 7 - Varredura de Ts -> instabilidade (LINEAR)    (Lab 11 - Parte 4)
%  Secao 8 - Zona morta do motor + compensacao inversa    (nao-linearidade real)
%  Secao 9 - Cenario REGULATORIO: rejeicao de perturbacao em alpha = 0
%
%  Forma ISA: C(s)=Kc(1 + 1/(Ti s) + Td s/(gamma Td s+1)), gamma=1/8
%  2-DOF (Lab 10): b pondera R no proporcional; c=0 (derivada so na medicao).
%
%  IMPORTANTE: a zona morta so entra nas Secoes 8 e 9 (dz_on=true). As Partes
%  2,3,4 sao LINEARES (dz_on=false), como no Lab do professor.
%% ============================================================================
clear all; close all; clc;

%% 1 - PROCESSO e ponto de operacao ------------------------------------------
Kp = 2.8968e-3;  zeta = 0.0743;  wn = 9.1069;
s = tf('s');
G = (Kp*wn^2)/(s^2 + 2*zeta*wn*s + wn^2);      % 2a ordem ressonante, SEM atraso
gamma = 1/8;   U_op = 127;  Y_op = 0;
fprintf('=== PROCESSO ===  G(s)=%.4f/(s^2+%.4f s+%.4f)  [Kp=%.2e zeta=%.3f wn=%.2f]\n\n',...
        Kp*wn^2, 2*zeta*wn, wn^2, Kp, zeta, wn);

%% 2 - BATERIA DE SINTONIA (Lab 09) ------------------------------------------
%  Planta 2a ordem RESSONANTE e SEM tempo morto: as tabelas empiricas FOPDT nao
%  se aplicam (forcar FOPDT -> Kc absurdo). Metodos sem derivativo instabilizam.
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
    if Td>0, Tf=gamma*Td; Ci=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
    else,    Ci=Kc*(1+1/(Ti*s)); end
    H=feedback(Ci*G,1); est = all(real(pole(H))<1e-6);
    warning('off','all'); [~,MF]=margin(Ci*G); warning('on','all');
    if est, S=stepinfo(H); mp=sprintf('%.0f',S.Overshoot); ts=sprintf('%.2f',S.SettlingTime);
    else,   mp='INST'; ts='INST'; end
    fprintf('%-30s %8.1f %8.3f %8.3f %9d %6.0f %7s %8s\n',metodos{i,1},Kc,Ti,Td,est,MF,mp,ts);
end
fprintf('-> Adotada a A (robusta): MF~53, sem overshoot, esforco moderado.\n\n');

%% 3 - SINTONIA ADOTADA + analise 2-DOF (Lab 10) -----------------------------
Kc = 57.31;  Ti = 0.1015;  Td = 0.4777;  Tf = gamma*Td;
b  = 0.7;    c  = 0;       Ts = 0.02;
C  = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
F  = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);
Hr = F*feedback(C*G,1);   Hu = F*feedback(C,G);   Hd = feedback(G,C);
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

%% 5 - LACO FOR: continuo x discreto  (LINEAR)  (Lab 11 - Parte 2) ------------
ref = 10;  tsim = 6;
[t5,y5,~] = malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tsim,ref,0,inf,130,75,255,false,false); % dz_on=false
tc=0:0.005:tsim; yc=rad2deg(deg2rad(ref)*step(Hr,tc));
[~,~,u5] = malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tsim,ref,0,inf,130,75,255,false,false);
figure('Name','Parte 2 - continuo x discreto');
subplot(2,1,1);
plot(tc,yc,'LineWidth',2); hold on; stairs(t5,y5,'LineWidth',1.4);
yline(ref,'--'); title('Lab 11 - Parte 2: continuo x discreto (linear)');
ylabel('\alpha (graus)'); legend('continuo','discreto (Tustin)','Location','SE'); grid on;
subplot(2,1,2);                                   % <<< acao de controle (estilo lab)
stairs(t5,u5,'k','LineWidth',1.2); ylabel('u (PWM)'); xlabel('t (s)'); grid on;
fprintf('=== Parte 2 ===  max|cont-disc| = %.3f graus (so o degrau do ZOH)\n\n', ...
        max(abs(interp1(tc,yc,t5)-y5)));

%% 6 - 2-DOF discreto: onde o peso b atua? (LINEAR) (Lab 11 - Parte 3) --------
%  Servo (degrau) em t=0 e perturbacao em t=3 s. b muda o SERVO, nao a regulacao.
%  Usa a sintonia RAPIDA B (tem overshoot) para EVIDENCIAR o efeito de b.
KcB=142.22; TiB=0.0452; TdB=0.3805; TfB=gamma*TdB;
[t6,yA,uA]=malha_furuta(Ts,0.5,KcB,TiB,TdB,TfB,gamma,3.5,10,40,2,130,75,255,false,false);
[~ ,yB,uB]=malha_furuta(Ts,1.0,KcB,TiB,TdB,TfB,gamma,3.5,10,40,2,130,75,255,false,false);
figure('Name','Parte 3 - 2-DOF');
subplot(2,1,1);
stairs(t6,yA,'LineWidth',1.5); hold on; stairs(t6,yB,'LineWidth',1.5);
yline(10,'--'); xline(2,':'); title('Lab 11 - Parte 3: b=0,5 x b=1,0 (servo t=0; perturbacao t=2s; sintonia B)');
ylabel('\alpha (graus)'); legend('b=0,5','b=1,0','Location','NE'); grid on;
subplot(2,1,2);                                   % <<< acao de controle (estilo lab)
stairs(t6,uA,'LineWidth',1.2); hold on; stairs(t6,uB,'LineWidth',1.2);
ylabel('u (PWM)'); xlabel('t (s)'); legend('b=0,5','b=1,0','Location','NE'); grid on;
iserv = t6<2;
fprintf('=== Parte 3 (2-DOF, sintonia B) ===\n');
fprintf('  Mp servo b=0,5: %.1f%% | b=1,0: %.1f%%  (b molda o servo)\n', ...
        100*max(0,max(yA(iserv))-10)/10, 100*max(0,max(yB(iserv))-10)/10);
fprintf('  max|yA-yB| APOS a perturbacao = %.3f graus (regulacao NAO ve b)\n\n', ...
        max(abs(yA(~iserv)-yB(~iserv))));

%% 7 - VARREDURA DE Ts -> instabilidade (LINEAR) (Lab 11 - Parte 4) ----------
%  Sintonia RAPIDA B (MF~40): ao crescer Ts a margem se esgota -> INSTAVEL.
bB=0.5;
figure('Name','Parte 4 - varredura de Ts'); hold on; cores=lines(5); i=0;
for Tsi=[0.02 0.05 0.10 0.15 0.20]
    i=i+1;
    [t7,y7,inst]=malha_furuta(Tsi,bB,KcB,TiB,TdB,TfB,gamma,4,10,0,inf,130,75,255,false,false);
    tag=''; if inst, tag=' (INSTAVEL)'; end
    stairs(t7,y7,'LineWidth',1.4,'Color',cores(i,:),'DisplayName',sprintf('Ts=%.0f ms%s',Tsi*1000,tag));
    fprintf('Parte 4: Ts=%3.0f ms | zD=%+.3f | %s\n',Tsi*1000,(2*TfB-Tsi)/(2*TfB+Tsi),ternario(inst,'INSTAVEL','estavel'));
end
yline(10,'--','DisplayName','ref'); ylim([-10 45]);
title('Lab 11 - Parte 4: varredura de Ts (sintonia B)'); ylabel('\alpha (graus)'); xlabel('t (s)');
legend('Location','NE'); grid on; fprintf('\n');

%% 7b - ESQUEMA MISTO (Tustin no I + Euler no D) x continuo -------------------
%  ESTA e' a lei que vai para o Arduino (pid_discreto). Ate aqui as figuras
%  usaram Tustin puro (malha_furuta); agora chamamos a lei MISTA do firmware.
par.Kc=Kc; par.Ti=Ti; par.Td=Td; par.Tf=Tf; par.b=b; par.c=c; par.Ts=Ts;
par.DZP=130; par.DZN=75; par.Umax=255; par.metodo_D='euler'; par.comp_dz=false;
[tm,ym,um] = malha_mista(par, 6, 10, 0, inf, false);
tc2=0:0.005:6; ycm=rad2deg(deg2rad(10)*step(Hr,tc2));
figure('Name','Misto x continuo');
subplot(2,1,1);
plot(tc2,ycm,'LineWidth',2); hold on; stairs(tm,ym,'LineWidth',1.4);
yline(10,'--'); title('Esquema MISTO (Tustin I + Euler D) x continuo — a lei do firmware');
ylabel('\alpha (graus)'); legend('continuo','discreto MISTO','Location','SE'); grid on;
subplot(2,1,2); stairs(tm,um,'k','LineWidth',1.2); ylabel('u (PWM)'); xlabel('t (s)'); grid on;
fprintf('=== Esquema MISTO ===  max|cont-misto| = %.3f graus\n\n', ...
        max(abs(interp1(tc2,ycm,tm)-ym)));

%% 7c - EULER x TUSTIN no derivativo: a PROVA do ringing (estilo Lab 12) ------
%  O ringing e' um fato sobre o PROPRIO filtro derivativo (nao sobre a malha
%  inteira - em Ts grande o atraso de amostragem degrada os dois metodos). Por
%  isso mostramos a resposta ao DEGRAU do termo D: D(z)=bD(z-1)/(z-aD).
%  Em Ts > 2*Tf: Tustin tem aD<0 -> saida ALTERNA de sinal (ringing 2Ts);
%                Euler  tem 0<aD<1 -> decai suave.
Ts_big = 0.15;                          % > 2*Tf = 119 ms
Nb = round(1.5/Ts_big); tb = (0:Nb-1)'*Ts_big; vD = ones(Nb,1);
udE = zeros(Nb,1); udT = zeros(Nb,1);
aDE = Tf/(Tf+Ts_big);            bDE = Kc*Td/(Tf+Ts_big);
aDT = (2*Tf-Ts_big)/(2*Tf+Ts_big); bDT = 2*Kc*Td/(2*Tf+Ts_big);
for k=1:Nb
    vp = (k>=2)*vD(max(k-1,1));
    udE(k) = aDE*((k>=2)*udE(max(k-1,1))) + bDE*(vD(k)-vp);
    udT(k) = aDT*((k>=2)*udT(max(k-1,1))) + bDT*(vD(k)-vp);
end
figure('Name','Euler x Tustin no filtro D');
stem(tb,udT,'r','filled','DisplayName',sprintf('Tustin (z_D=%+.2f<0): RINGING',aDT)); hold on;
plot(tb,udE,'b-o','LineWidth',1.8,'DisplayName',sprintf('Euler (z_D=%+.2f>0): suave',aDE));
yline(0,'k'); title(sprintf('Prova: resposta do filtro DERIVATIVO ao degrau (Ts=%.0f ms > 2Tf)',Ts_big*1000));
ylabel('u_D  (saida do termo derivativo)'); xlabel('t (s)'); legend('Location','NE'); grid on;
fprintf('=== Euler x Tustin no filtro D (Ts=%.0f ms) ===\n',Ts_big*1000);
fprintf('  Tustin ud(1:4) = [%.1f %.1f %.1f %.1f]  <- ALTERNA (ringing)\n',udT(1),udT(2),udT(3),udT(4));
fprintf('  Euler  ud(1:4) = [%.1f %.1f %.1f %.1f]  <- decai suave\n\n',udE(1),udE(2),udE(3),udE(4));

%% 8 - ZONA MORTA do motor + compensacao inversa (dz_on=true) -----------------
DZP=130; DZN=75; Umax=255;
[t8c,y8c,~]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,6,10,0,inf,DZP,DZN,Umax,true ,true);  % COM comp
[t8s,y8s,~]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,6,10,0,inf,DZP,DZN,Umax,false,true);  % SEM comp
figure('Name','Zona morta');
stairs(t8s,y8s,'r','LineWidth',1.5); hold on; stairs(t8c,y8c,'b','LineWidth',1.7); yline(10,'--');
title('Zona morta assimetrica (+130/-75 PWM): sem x com compensacao');
ylabel('\alpha (graus)'); xlabel('t (s)'); legend('SEM compensacao','COM compensacao','Location','SE'); grid on;
fprintf('=== ZONA MORTA ===  compensacao inversa OBRIGATORIA (esforco ~ banda morta)\n\n');

%% 9 - CENARIO REGULATORIO: rejeicao de perturbacao em alpha = 0 (dz_on=true) -
%  Principal (0 graus). Para outro ponto, troque ref_deg (ex.: 5).
ref_deg = 0;  d_pwm = 40;  t_dist = 2.0;  tfim = 8;
[t9,y9,u9]=malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tfim,ref_deg,d_pwm,t_dist,DZP,DZN,Umax,true,true);
[~ ,y9o]  =malha_furuta_ol(Ts,tfim,d_pwm,t_dist,G);
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
function [t,y,u] = malha_mista(par, tsim, ref_deg, d_pwm, t_dist, dz_on)
% Malha fechada discreta que CHAMA a lei do firmware (pid_discreto, esquema
% MISTO Tustin-I + Euler-D). Planta linear G(s) discretizada por ZOH.
    Kp=2.8968e-3; zeta=0.0743; wn=9.1069; s=tf('s');
    Gd=c2d((Kp*wn^2)/(s^2+2*zeta*wn*s+wn^2), par.Ts, 'zoh');
    [nb,na]=tfdata(Gd,'v'); nb=nb/na(1); na=na/na(1); b1=nb(2); b2=nb(3);
    N=round(tsim/par.Ts); t=(0:N-1)'*par.Ts;
    dpert=zeros(N,1); if isfinite(t_dist), dpert(t>=t_dist)=d_pwm; end
    y=zeros(N,1); u=zeros(N,1); ueff=zeros(N,1);
    est.I=0; est.e_ant=0; est.v_ant=0; est.ud=0;   % estados do PID
    r=deg2rad(ref_deg);
    for k=1:N
        y1=(k>=2)*y(max(k-1,1)); y2=(k>=3)*y(max(k-2,1));
        u1=(k>=2)*ueff(max(k-1,1)); u2=(k>=3)*ueff(max(k-2,1));
        d1=(k>=2)*dpert(max(k-1,1)); d2=(k>=3)*dpert(max(k-2,1));
        y(k)=-na(2)*y1-na(3)*y2 + b1*(u1+d1) + b2*(u2+d2);
        [uk,est]=pid_discreto(r, y(k), est, par);  % <<< a lei do Arduino
        u(k)=uk;
        if dz_on, ueff(k)=deadzone(uk,par.DZP,par.DZN); else, ueff(k)=uk; end
    end
    y=rad2deg(y);
end

function v = pega(x,k)
    if k>=1, v=x(k); else, v=0; end
end
function r = ternario(cond,a,b)
    if cond, r=a; else, r=b; end
end

function [t,y,u,inst] = malha_furuta(Ts,b,Kc,Ti,Td,Tf,gamma,tsim,ref_deg, ...
                          d_pwm,t_dist, DZP,DZN,Umax, comp, dz_on)
% Malha fechada discreta do Furuta (2a ordem, SEM atraso), padrao Lab 11:
%   R -> F(z) -> (soma +-) -> C(z) -> saturacao -> [comp DZ + DZ motor] -> planta
% dz_on = false  -> malha LINEAR (sem zona morta): Partes 2, 3 e 4.
% dz_on = true   -> inclui a zona morta do motor (Secoes 8 e 9); comp liga a
%                   compensacao inversa.  c=0 (derivada so na medicao).
    s=tf('s');
    Cc=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
    Ff=tf([b*Ti*Tf, b*Ti+Tf, 1],[Ti*Td*(1+gamma), Ti+Tf, 1]);
    Cz=c2d(Cc,Ts,'tustin'); [nC,dC]=tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
    Fz=c2d(Ff,Ts,'tustin'); [nF,dF]=tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
    Kp=2.8968e-3; zeta=0.0743; wn=9.1069;
    Gd=c2d((Kp*wn^2)/(s^2+2*zeta*wn*s+wn^2),Ts,'zoh');
    [nb,na]=tfdata(Gd,'v'); nb=nb/na(1); na=na/na(1); b1=nb(2); b2=nb(3);
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
        if dz_on
            if comp, ucmd(k)=inv_deadzone(v(k),DZP,DZN); else, ucmd(k)=v(k); end
            ucmd(k)=min(max(ucmd(k),-Umax),Umax);
            ueff(k)=deadzone(ucmd(k),DZP,DZN);
        else
            ueff(k)=v(k);                 % LINEAR: sem zona morta
        end
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
% ===========================================================================
%  PID 2-DOF DISCRETO - FORMA 3 TERMOS - ESTA E' A EQUACAO QUE VAI PARA O ARDUINO
%
%  DISCRETIZACAO MISTA (escolha de projeto - ver relatorio):
%    * INTEGRAL  -> TUSTIN (trapezio):  Ki = Kc*Ts/(2*Ti)
%         I(k) = I(k-1) + Ki*( e(k) + e(k-1) )
%      Motivo: o trapezio e' de 2a ordem (mais preciso que Euler) e mapeia o
%      integrador exatamente em z=1 (erro nulo em regime permanente garantido).
%
%    * DERIVATIVO -> EULER (backward / de tras para frente):
%         aD = Tf/(Tf+Ts)          bD = Kc*Td/(Tf+Ts)
%         ud(k) = aD*ud(k-1) + bD*( vD(k) - vD(k-1) ),   vD = c*r - y  (c=0 -> -y)
%      Motivo: o polo do filtro derivativo fica aD = Tf/(Tf+Ts), que e' SEMPRE
%      0 < aD < 1 para qualquer Ts. Com Tustin, aD = (2Tf-Ts)/(2Tf+Ts) fica
%      NEGATIVO quando Ts > 2*Tf (aqui, Ts > 119 ms), produzindo RINGING
%      (oscilacao de periodo 2Ts) no termo derivativo. Euler elimina esse risco
%      e deixa o firmware robusto a variacoes/jitter do periodo de amostragem.
%
%  P com ponderacao b | saturacao + compensacao de zona morta DENTRO do
%  controlador | anti-windup por clamping.
%  Estados: est.I, est.e_ant, est.v_ant, est.ud
% ===========================================================================
    e  = r - y;
    P  = par.Kc*(par.b*r - y);          % proporcional com peso b
    vD = par.c*r - y;                    % c = 0  ->  derivada so na medicao

    if ~isfield(par,'metodo_D') || strcmp(par.metodo_D,'euler')
        aD = par.Tf/(par.Tf + par.Ts);              % EULER: sempre 0<aD<1
        bD = par.Kc*par.Td/(par.Tf + par.Ts);
    else                                            % TUSTIN (para comparacao)
        aD = (2*par.Tf - par.Ts)/(2*par.Tf + par.Ts);   % fica <0 se Ts>2Tf -> ringing
        bD = 2*par.Kc*par.Td/(2*par.Tf + par.Ts);
    end
    est.ud = aD*est.ud + bD*(vD - est.v_ant);   % derivativo filtrado

    Ki     = par.Kc*par.Ts/(2*par.Ti);          % TUSTIN (trapezio)
    I_cand = est.I + Ki*(e + est.e_ant);

    u_ns = P + I_cand + est.ud;                  % antes de saturar

    % compensacao inversa da zona morta (assimetrica).
    % par.comp_dz=false desliga (usado nas figuras LINEARES, secoes 7b/7c).
    if isfield(par,'comp_dz') && ~par.comp_dz
        u_cmd = u_ns;
    elseif u_ns >  1e-9, u_cmd = u_ns + par.DZP;
    elseif u_ns < -1e-9, u_cmd = u_ns - par.DZN;
    else,                u_cmd = 0;
    end
    u = min(max(u_cmd, -par.Umax), par.Umax);    % saturacao fisica do PWM

    % anti-windup por clamping: so integra se nao estiver saturado empurrando mais fundo
    if ~((u ~= u_cmd) && (sign(e) == sign(u_ns)))
        est.I = I_cand;
    end
    est.e_ant = e;  est.v_ant = vD;
end
