%% ============================================================================
%  TRABALHO FINAL - Controle PID 2-DOF do Pendulo de Furuta (posicao INFERIOR)
%  Disciplina: Introducao aos Sistemas de Controle - 2026/1
%  Prof. Me. Daniel Miranda Cruz  |  UFMT/FAENG - Eng. de Computacao
%  Grupo: Daniel M. Zamarioli, Erick G. B. Sebba, Fabio C. Barbosa Jr.
%
%  CENARIO: REGULATORIO. O pendulo opera para baixo, mantido em alpha = 0 graus.
%  O objetivo de controle e' manter Y_op = 0 e REJEITAR PERTURBACOES; a capacidade
%  de SEGUIMENTO de referencia e' demonstrada como recurso (degrau pequeno).
%
%  Este script entrega os itens pedidos:
%   (1) funcionamento em simulacao;               -> secoes 4, 7 e 8
%   (2) processo de ajuste e Kc,Ti,Td,b,c,Ts;     -> secoes 3, 5 e 6
%   (3) PID 2-DOF DISCRETIZADO em codigo (Ts);     -> secoes 6, 7 e pid_discreto()
%   (4) seguimento + rejeicao de perturbacao.      -> secoes 7 e 8
%
%  Forma do PID (padrao ISA do curso, derivada filtrada, gamma = 1/8):
%     C(s) = Kc*( 1 + 1/(Ti*s) + Td*s/(gamma*Td*s + 1) )
%  2-DOF (Lab 10):  u = Kc[(b*r - y) + (1/Ti)INT(r-y) + D_filt*(c*r - y)]
%  Roda sozinho pelas equacoes a diferencas (Lab 11/12). O modelo Simulink
%  equivalente e' criado por build_modelo_furuta.m (secao 9).
%% ============================================================================
clear; close all; clc;

%% 1 - PROCESSO (modelo identificado na Entrega 2) --------------------------
Kp   = 2.8968e-3;      % ganho estatico (rad/PWM)  -- DEADZONE embutida na ID
zeta = 0.0743;         % coef. de amortecimento
wn   = 9.1069;         % freq. natural (rad/s)
s = tf('s');
G = (Kp*wn^2) / (s^2 + 2*zeta*wn*s + wn^2);   % PWM -> alpha (agrupa Gm*Gp)

fprintf('=== PROCESSO (Entrega 2) ===\n');
fprintf('  G(s) = %.4f / (s^2 + %.4f s + %.4f)\n', Kp*wn^2, 2*zeta*wn, wn^2);
fprintf('  Kp_DC=%.3e  zeta=%.4f  wn=%.4f rad/s | polos MA: %.2f +/- %.2fj\n\n', ...
        Kp, zeta, wn, -zeta*wn, wn*sqrt(1-zeta^2));

%% 2 - PONTO DE OPERACAO -----------------------------------------------------
U_op = 127;    % ~50% do PWM (bias do braco)
Y_op = 0;      % 0 graus (pendulo na vertical inferior) -- setpoint do regulador
alpha_ref_deg = 0;   % CENARIO REGULATORIO

%% 3 - SINTONIA ESCOLHIDA (regulatoria, robusta) -----------------------------
%  Metodo: loop-shaping/ITAE com restricao de margem de fase (>= ~55 deg) e de
%  esforco. As tabelas empiricas FOPDT (ZN/CHR/Cohen-Coon) NAO se aplicam, pois
%  a planta e' de 2a ordem ressonante e sem tempo morto (ver relatorio).
gamma = 1/8;                 % filtro derivativo (fixo na disciplina)
Kc = 57.31;   Ti = 0.1015;   Td = 0.4777;   Tf = gamma*Td;   % Tf = 0.05971 s
b  = 0.7;     c  = 0;        % 2-DOF: c=0 -> derivada so na medicao
Ts = 0.02;                   % periodo de amostragem (50 Hz, = firmware Entrega 2)

% ---- Sintonia alternativa "rapida" (comparacao no relatorio) ----
%  Kc=142.22; Ti=0.0452; Td=0.3805; b=0.5;  (MF~40 deg, Ts~0.6s, esforco alto)

fprintf('=== SINTONIA (forma ISA) ===\n');
fprintf('  Kc=%.2f | Ti=%.4f s | Td=%.4f s | Tf=%.5f s | b=%.2f | c=%.0f | Ts=%.3f s\n\n',...
        Kc,Ti,Td,Tf,b,c,Ts);

%% 4 - ANALISE EM MALHA FECHADA (contínua, linear) ---------------------------
C = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s + 1));    % controlador de realimentacao completo
Hr = feedback(C*G, 1);      % R -> Y  (seguimento, 1-DOF)
Hu = feedback(C, G);        % R -> U  (esforco)
Hd = feedback(G, C);        % D(entrada) -> Y  (rejeicao de perturbacao)

[Gm,Pm,~,Wcp] = margin(C*G);
Sr = stepinfo(Hr);
fprintf('=== MALHA FECHADA (linear) ===\n');
fprintf('  Margem de fase = %.1f deg | crossover = %.2f rad/s | MG = %.1f dB\n', ...
        Pm, Wcp, 20*log10(Gm));
fprintf('  Seguimento:  Mp=%.1f%% | Tr=%.2f s | Ts(2%%)=%.2f s\n', ...
        Sr.Overshoot, Sr.RiseTime, Sr.SettlingTime);
fprintf('  Perturbacao: pico=%.2f graus (degrau de 20 PWM na entrada)\n\n', ...
        rad2deg(20*max(abs(step(Hd, 0:0.01:8)))));

%% 5 - DEADZONE do motor (medida) + COMPENSACAO ------------------------------
%  Medido: sentido + (anti-horario) parte em ~120-140 PWM (~2,5-2,7 V);
%          sentido - (horario)      parte em ~ 70-80  PWM (~-1,3 -1,5 V).
DZP = 130;   DZN = 75;      % limiares de partida (assimetricos), em PWM
Umax = 255;                 % saturacao fisica do PWM
fprintf('=== DEADZONE (assimetrica) ===  +%d / -%d PWM  -> compensacao inversa ON\n\n', DZP, DZN);

%% 6 - DISCRETIZACAO (Tustin) -> equacao a diferencas ------------------------
%  Controlador C(z) (age sobre o erro) e pre-filtro F(z) (carrega o peso b).
Cz = c2d(C, Ts, 'tustin');
F  = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);   % F(0)=1
Fz = c2d(F, Ts, 'tustin');

[numCz,denCz] = tfdata(Cz,'v'); numCz=numCz/denCz(1); denCz=denCz/denCz(1);
[numFz,denFz] = tfdata(Fz,'v'); numFz=numFz/denFz(1); denFz=denFz/denFz(1);

% Verificacao das regras de Ts (Lab 11/12)
zD = (2*Tf - Ts)/(2*Tf + Ts);      % polo do filtro D (Tustin): > 0 sem ringing
fprintf('=== DISCRETIZACAO (Tustin, Ts=%.0f ms) ===\n', Ts*1000);
fprintf('  C(z): q0=%.4f q1=%.4f q2=%.4f | den: 1 %.4f %.4f\n', ...
        numCz(1),numCz(2),numCz(3),denCz(2),denCz(3));
fprintf('  F(z): n0=%.5f n1=%.5f n2=%.5f | den: 1 %.4f %.4f\n', ...
        numFz(1),numFz(2),numFz(3),denFz(2),denFz(3));
fprintf('  Regras de Ts: 2*Tf=%.0f ms | Tac/30=%.0f ms | 2pi/wn/10=%.0f ms | zD=%+.3f (OK se >0)\n\n', ...
        2*Tf*1000, Sr.SettlingTime/30*1000, 2*pi/wn/10*1000, zD);

% Planta discreta (ZOH) para o laco a diferencas
Gz = c2d(G, Ts, 'zoh'); [numGz,denGz]=tfdata(Gz,'v'); numGz=numGz/denGz(1); denGz=denGz/denGz(1);

%% 7 - SIMULACAO REGULATORIA (r=0, degrau de perturbacao) --------------------
%  Laco tipo firmware (Lab 12): PID discreto + compensacao de deadzone + AW.
tfim = 8;  t_dist = 2.0;  d_pwm = 40;    % perturbacao de +40 PWM na entrada em t=2s
[t7, y7, u7] = malha_discreta(Ts, tfim, 0, d_pwm, t_dist, ...
                 Kc,Ti,Td,Tf,gamma,b,c, DZP,DZN,Umax, numGz,denGz, true);
% versao SEM controle (referencia visual)
[~ , y7ol, ~] = malha_discreta(Ts, tfim, 0, d_pwm, t_dist, ...
                 Kc,Ti,Td,Tf,gamma,b,c, DZP,DZN,Umax, numGz,denGz, false);

figure('Name','Regulatorio');
subplot(2,1,1);
plot(t7, y7ol, 'r', 'LineWidth',1.4); hold on;
plot(t7, y7,  'b', 'LineWidth',2);
yline(0,'--','Color',[.4 .4 .4]); xline(t_dist,'k:');
title('Cenario REGULATORIO: rejeicao de perturbacao em \alpha = 0 (degrau 40 PWM em t=2 s)');
ylabel('\alpha (graus)'); legend('sem controle','com PID 2-DOF','Location','best'); grid on;
subplot(2,1,2);
stairs(t7, u7, 'b', 'LineWidth',1.3); hold on;
yline(DZP,':','Color',[.4 .4 .4]); yline(-DZN,':','Color',[.4 .4 .4]);
title('Comando do PID (PWM)'); ylabel('u (PWM)'); xlabel('Tempo (s)'); grid on;

fprintf('=== REGULATORIO ===  pico apos perturbacao = %.2f graus | alpha final = %.3f graus\n\n', ...
        max(abs(y7)), y7(end));

%% 8 - SIMULACAO SERVO (capacidade de seguimento + efeito de b) --------------
%  Recurso: mesmo operando em 0, o controlador segue pequenos degraus.
figure('Name','Servo');
tsv=5; hold on;
for bb = [1.0 0.7 0.4]
    [tt, yy] = malha_discreta(Ts, tsv, 5, 0, inf, ...
                 Kc,Ti,Td,Tf,gamma,bb,c, DZP,DZN,Umax, numGz,denGz, true);
    plot(tt, yy, 'LineWidth',1.8, 'DisplayName',sprintf('b=%.1f',bb));
end
yline(5,'--','Color',[.4 .4 .4],'DisplayName','ref (5 graus)');
title('Capacidade de seguimento (servo) e efeito do peso b'); ...
ylabel('\alpha (graus)'); xlabel('Tempo (s)'); legend('Location','best'); grid on;

%% 9 - (OPCIONAL) modelo Simulink equivalente --------------------------------
%  Rode uma vez para criar o modelo e depois simule:
%     build_modelo_furuta;                % cria modelo_furuta.slx
%     out = sim('modelo_furuta');
%     figure; plot(out.tout, out.alpha_deg);

fprintf('Script finalizado. Ajuste a sintonia na secao 3 e re-execute.\n');

%% ======================= FUNCOES LOCAIS ====================================
function [t, y, u] = malha_discreta(Ts, tfim, ref_deg, d_pwm, t_dist, ...
              Kc,Ti,Td,Tf,gamma,b,c, DZP,DZN,Umax, numGz,denGz, controla)
% Malha fechada discreta (padrao Lab 11/12): pre-filtro F(z) + C(z) sobre o erro,
% compensacao inversa da deadzone e planta discreta ZOH. Perturbacao na entrada.
    s = tf('s');
    C = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
    F = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);
    Cz=c2d(C,Ts,'tustin'); [nC,dC]=tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
    Fz=c2d(F,Ts,'tustin'); [nF,dF]=tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
    a1=denGz(2); a2=denGz(3); b1=numGz(2); b2=numGz(3);   % y(k)=-a1 y1 -a2 y2 + b1 u1 + b2 u2

    N=round(tfim/Ts); t=(0:N-1)'*Ts;
    r = deg2rad(ref_deg)*ones(N,1);
    dpert = d_pwm*(t >= t_dist);
    rf=zeros(N,1); e=zeros(N,1); v=zeros(N,1); ucmd=zeros(N,1); ueff=zeros(N,1); yr=zeros(N,1);
    pg=@(x,k) (k>=1)*x(max(k,1));
    for k=1:N
        yr(k) = -a1*pg(yr,k-1) - a2*pg(yr,k-2) ...
                + b1*(pg(ueff,k-1)+pg(dpert,k-1)) + b2*(pg(ueff,k-2)+pg(dpert,k-2));
        if controla
            rf(k)= -dF(2)*pg(rf,k-1) -dF(3)*pg(rf,k-2) + nF(1)*r(k)+nF(2)*pg(r,k-1)+nF(3)*pg(r,k-2);
            e(k) = rf(k) - yr(k);
            v(k) = -dC(2)*pg(v,k-1) -dC(3)*pg(v,k-2) + nC(1)*e(k)+nC(2)*pg(e,k-1)+nC(3)*pg(e,k-2);
            v(k) = min(max(v(k),-Umax),Umax);
            ucmd(k) = inv_deadzone(v(k), DZP, DZN);       % compensacao inversa
            ucmd(k) = min(max(ucmd(k),-Umax),Umax);
            ueff(k) = deadzone(ucmd(k), DZP, DZN);        % nao-linearidade do motor
        end
    end
    y = rad2deg(yr); u = v;
end

function ue = deadzone(u, DZP, DZN)
    if u >  DZP,      ue = u - DZP;
    elseif u < -DZN,  ue = u + DZN;
    else,             ue = 0;
    end
end

function uc = inv_deadzone(v, DZP, DZN)
    if v >  1e-9,     uc = v + DZP;
    elseif v < -1e-9, uc = v - DZN;
    else,             uc = 0;
    end
end

function [u, est] = pid_discreto(r, y, est, par)
% ===========================================================================
%  PID 2-DOF DISCRETO - FORMA 3 TERMOS, PRONTA PARA VIRAR FIRMWARE (Arduino).
%  Chamada a cada Ts. Acoes: P com peso b | I trapezoidal (Tustin) com clamping
%  | D filtrada (Tf=gamma*Td) sobre -y (c=0). Saturacao + compensacao de deadzone
%  DENTRO do controlador. Estados: est.I, est.e_ant, est.v_ant, est.ud.
% ===========================================================================
    e = r - y;                                   % erro (alimenta o integral)
    P = par.Kc*(par.b*r - y);                     % P com ponderacao da referencia
    vD = -y;                                       % c=0: derivada so na medicao
    aD = (2*par.Tf - par.Ts)/(2*par.Tf + par.Ts);  % filtro D (Tustin)
    bD = 2*par.Kc*par.Td/(2*par.Tf + par.Ts);
    est.ud = aD*est.ud + bD*(vD - est.v_ant);      % termo D filtrado
    I_cand = est.I + (par.Kc*par.Ts/(2*par.Ti))*(e + est.e_ant);   % I trapezoidal
    u_ns = P + I_cand + est.ud;                    % antes de saturar
    % --- compensacao inversa da deadzone ---
    if     u_ns >  1e-9, u_cmd = u_ns + par.DZP;
    elseif u_ns < -1e-9, u_cmd = u_ns - par.DZN;
    else,                u_cmd = 0;
    end
    u = min(max(u_cmd, -par.Umax), par.Umax);      % saturacao fisica
    % --- anti-windup por clamping ---
    if (u ~= u_cmd) && (sign(e) == sign(u_ns))
        % saturou e o erro empurraria mais fundo -> congela o integrador
    else
        est.I = I_cand;
    end
    est.e_ant = e;  est.v_ant = vD;
end
