%% Aula 24 - Lab 11: PID discreto (equacao a diferencas)
% UFMT/FAENG/DCA - Engenharia de Computacao
% Introducao aos Sistemas de Controle (2026/1) - Prof. Me. Daniel Miranda Cruz
%
% Objetivo: transformar o PID 2-DOF continuo do Bloco 3 (C(s) e F(s)) em
% DUAS equacoes a diferencas e fechar a malha inteira num laco for, como
% na Aula 01 - so que agora quem vira codigo eh o CONTROLADOR.
%
% Pontos-chave:
%  - Discretizacao por TUSTIN: c2d(.,Ts,'tustin') -> tfdata -> coeficientes
%  - Atraso de transporte EXATO no discreto: u(k-1-d), com d = theta/Ts inteiro
%    (nada de Pade! por isso usamos Ts em fracoes de tau)
%  - Processo discretizado de forma exata: a1 = exp(-Ts/tau), b1 = Kp*(1-a1)
%
% ---------------------------------------------------------------------------
% CONTRAPROVA SIMULINK (opcional) - estrutura do modelo aula_11.slx:
%   Ramo discreto: Step (r) -> Discrete Transfer Fcn F(z) [numFz/denFz, Ts]
%     -> Sum(+-) -> Discrete Transfer Fcn C(z) [numCz/denCz, Ts]
%     -> Zero-Order Hold (Ts) -> Transfer Fcn Kp/(tau.s+1) -> Transport Delay (theta)
%     -> Sum de perturbacao (+ d_as via ganho -3.2653) -> y -> realimenta o Sum(+-)
%   Ramo continuo em paralelo (F(s), C(s), mesma planta) para sobreposicao.
%   Exportar: To Workspace (y_d, y_c, u_d) via Mux. Solver: ode45, tsim 2500 s.
% ---------------------------------------------------------------------------

clear all; close all; clc;

%% 1 - Inicializacao: processo, sintonia oficial e ponderacoes 2-DOF
% Processo (tanque linearizado + atraso, cenario do Lab 09)
Kp    = 2.721;        % m/abertura
tau   = 27.21;        % s
theta = 2*tau;        % s  (cenario theta = 2*tau do Lab 09)
Kp_as = -3.2653;      % ganho da perturbacao d_as (valvula de saida), sem atraso

% Sintonia ITAE regulatorio (Lopez) - Lab 09
Kc = 0.2587;
Ti = 53.90;           % s
Td = 20.66;           % s
gamma = 1/8;
Tf = gamma*Td;        % filtro da derivativa

% Ponderacoes 2-DOF (Lab 10 -> valores oficiais do curso)
b = 0.5;
c = 0;                % derivada so na medicao

% Controlador e pre-filtro continuos (A19 e A21)
s = tf('s');
C = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
F = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);   % F(0)=1

% Planta com atraso interno EXATO (o MATLAB simula sem aproximacao)
G = Kp/(tau*s+1);
G.InputDelay = theta;
%% ── Parâmetros do PROCESSO ────────────────────────────────────────────
A  = 0.5;       % m² - área da seção transversal
ke = 0.05;      % m³/s - coeficiente da válvula de entrada
ks = 0.015;     % m³/s - coeficiente da válvula de saída
ae0 = 0.6;      % abertura de entrada no ponto de operação
as0 = 0.5;      % abertura de saída no ponto de operação
g   = 9.8;      % m/s²

%% ── Ponto de operação ─────────────────────────────────────────────────
H0 = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2

%% ── Linearização em torno de (H0, ae0, as0) ──────────────────────────
% dh/dt = (1/A)*[ke*ae - ks*as*sqrt(2*g*h)]
% Expandindo em série de Taylor (1ª ordem):
%   dδh/dt = -(alpha)*δh + (beta_ae)*δae
%
alpha   = (ks*as0*sqrt(2*g)) / (2*A*sqrt(H0));   % 1/tau
beta_ae = ke / A;

tau_proc = 1 / alpha;
Kp_proc  = beta_ae / alpha;

fprintf('\n--- Parâmetros do modelo linearizado ---\n');
fprintf('tau_proc = %.2f s\n', tau_proc);
fprintf('Kp_proc  = %.4f m/abert.\n', Kp_proc);

s = tf('s');
G = Kp_proc / (tau_proc*s + 1)

alpha   = -(1/A) * ks * as0 * g / sqrt(2*g*H0);
beta_ae =  ke / A;
beta_as = -(1/A) * ks * sqrt(2*g*H0);

% Constante de tempo e ganhos da FT
tau   = -1/alpha;
Kp_ae = -beta_ae/alpha;
Kp_as = -beta_as/alpha;

G_ae = Kp_ae / (tau*s + 1);     % FT da abertura de entrada para o nivel
G_as = Kp_as / (tau*s + 1);     % FT da abertura de saida para o nivel


%% 2 - Referencia continua: malha 2-DOF exata
tsim = 2500;
tc   = 0:0.5:tsim;
Hr   = F*feedback(C*G, 1);          % y/r do 2-DOF
yc   = step(0.1*Hr, tc);            % degrau servo de +0,1 m
S    = stepinfo(yc, tc, 0.1, 'SettlingTimeThreshold', 0.02)
% Esperado: Mp ~ 1%, Tac(2%) ~ 222 s  -> e a regra escondida da Aula 01:
Ts_regra = S.SettlingTime/30        % ~7,4 s ... guardem esse numero p/ A25!

%% 3 - Parte 1: discretizacao Tustin e coeficientes
Ts = tau/10                          % = 2,721 s
d  = theta/Ts                        % = 20 amostras EXATAS de atraso

Cz = c2d(C, Ts, 'tustin');
Fz = c2d(F, Ts, 'tustin');

[numCz, denCz] = tfdata(Cz, 'v');
numCz = numCz/denCz(1);  denCz = denCz/denCz(1);   % den(1) = 1 SEMPRE!
[numFz, denFz] = tfdata(Fz, 'v');
numFz = numFz/denFz(1);  denFz = denFz/denFz(1);

fprintf('\nC(z):  q0=%.4f  q1=%.4f  q2=%.4f | den: 1  %.4f  %.4f\n', ...
        numCz(1), numCz(2), numCz(3), denCz(2), denCz(3));
fprintf('F(z):  n0=%.4f  n1=%.4f  n2=%.4f | den: 1  %.4f  %.4f\n', ...
        numFz(1), numFz(2), numFz(3), denFz(2), denFz(3));
polos_Cz = roots(denCz)              % um deles em z = 1 (o integrador!)

%% 4 - Parte 2: a malha inteira num laco for (padrao Aula 01)
% Processo discreto exato (ZOH na entrada): y(k)=a1*y(k-1)+b1*u(k-1-d)
a1  = exp(-Ts/tau);
b1  = Kp*(1-a1);
b1d = Kp_as*(1-a1);                  % canal da perturbacao (sem atraso)

N  = round(tsim/Ts);
t  = (0:N-1)*Ts;
r  = 0.1*ones(1,N);                  % degrau servo
das = zeros(1,N);                    % sem perturbacao na Parte 2

% vetores e estados (os "valores que sobrevivem na memoria")
rf = zeros(1,N); e = zeros(1,N); u = zeros(1,N); y = zeros(1,N);
for k = 1:N
    % --- indices passados com protecao no inicio ---
    rf1 = pega(rf,k-1); rf2 = pega(rf,k-2);
    r1  = pega(r ,k-1); r2  = pega(r ,k-2);
    e1  = pega(e ,k-1); e2  = pega(e ,k-2);
    u1  = pega(u ,k-1); u2  = pega(u ,k-2);
    y1  = pega(y ,k-1);
    ud  = pega(u ,k-1-d);            % <<< o atraso: uma linha, EXATO
    dd  = pega(das,k-1);

    % --- processo responde ao controle de d+1 amostras atras ---
    y(k) = a1*y1 + b1*ud + b1d*dd;

    % --- controlador 2-DOF: F(z) filtra a referencia... ---
    rf(k) = -denFz(2)*rf1 - denFz(3)*rf2 ...
            + numFz(1)*r(k) + numFz(2)*r1 + numFz(3)*r2;
    % --- ...e C(z) age sobre o erro filtrado ---
    e(k) = rf(k) - y(k);
    u(k) = -denCz(2)*u1 - denCz(3)*u2 ...
           + numCz(1)*e(k) + numCz(2)*e1 + numCz(3)*e2;
end

% Sobreposicao continuo x discreto
yc_i = interp1(tc, yc, t);
fprintf('\nParte 2: max|y_cont - y_disc| = %.4f m\n', max(abs(yc_i - y)));

figure(1);
subplot(2,1,1);
plot(tc, yc, 'LineWidth', 2); hold on;
stairs(t, y, 'LineWidth', 1.5);
title('Lab 11 - Parte 2: malha continua x malha discreta (Ts = \tau/10)');
ylabel('\DeltaH [m]'); legend('continuo','discreto','Location','southeast'); grid on;
subplot(2,1,2);
stairs(t, u, 'LineWidth', 1.5);
ylabel('\Deltaa_e [abertura]'); xlabel('Tempo [s]'); grid on;

%% 5 - Parte 3: 2-DOF discreto - b mexe onde?
% Ensaio: degrau servo +0,1 m em t=0 e perturbacao d_as=+0,05 em t=1200 s
[t3, yA, uA] = malha_discreta(Ts, 0.5, Kp, tau, theta, Kp_as, Kc, Ti, Td, Tf, gamma, tsim, 0.1, 0.05, 1200);
[~ , yB, uB] = malha_discreta(Ts, 1.0, Kp, tau, theta, Kp_as, Kc, Ti, Td, Tf, gamma, tsim, 0.1, 0.05, 1200);

figure(2);
subplot(2,1,1);
stairs(t3, yA, 'LineWidth', 1.5); hold on; stairs(t3, yB, 'LineWidth', 1.5);
title('Lab 11 - Parte 3: b = 0,5 x b = 1,0 (servo em t=0; perturbacao em t=1200 s)');
ylabel('\DeltaH [m]'); legend('b = 0,5','b = 1,0','Location','southeast'); grid on;
subplot(2,1,2);
stairs(t3, uA, 'LineWidth', 1.5); hold on; stairs(t3, uB, 'LineWidth', 1.5);
ylabel('\Deltaa_e'); xlabel('Tempo [s]'); grid on;

i_servo = t3 < 1200;
fprintf('Parte 3: Mp servo b=0,5: %.2f%% | b=1,0: %.2f%%\n', ...
        100*max(0,(max(yA(i_servo))-0.1))/0.1, 100*max(0,(max(yB(i_servo))-0.1))/0.1);
fprintf('Parte 3: max|yA - yB| APOS a perturbacao = %.2e (regulacao nao enxerga b!)\n', ...
        max(abs(yA(~i_servo) - yB(~i_servo))));

%% 6 - Parte 4: primeiro esbarrao no limite de Ts (gancho p/ Lab 12)
lista_Ts = [tau/10, tau/5, tau/2];
figure(3); hold on; cores = lines(numel(lista_Ts));
for i = 1:numel(lista_Ts)
    Tsi = lista_Ts(i);
    [t4, y4, ~, estourou] = malha_discreta(Tsi, 0.5, Kp, tau, theta, Kp_as, ...
                                           Kc, Ti, Td, Tf, gamma, tsim, 0.1, 0, inf);
    if estourou
        fprintf('Parte 4: Ts = %.3f s (d = %d)  ->  INSTAVEL! (simulacao interrompida)\n', ...
                Tsi, round(theta/Tsi));
        stairs(t4, y4, 'LineWidth', 1.5, 'Color', cores(i,:), ...
               'DisplayName', sprintf('Ts = \\tau/%g  (INSTAVEL)', tau/Tsi));
    else
        Mp = 100*max(0,(max(y4)-0.1))/0.1;
        fprintf('Parte 4: Ts = %.3f s (d = %d)  ->  estavel, Mp = %.2f%%\n', ...
                Tsi, round(theta/Tsi), Mp);
        stairs(t4, y4, 'LineWidth', 1.5, 'Color', cores(i,:), ...
               'DisplayName', sprintf('Ts = \\tau/%g', tau/Tsi));
    end
end
plot(tc, yc, 'k--', 'LineWidth', 1, 'DisplayName', 'continuo');
ylim([-0.05 0.3]);
title('Lab 11 - Parte 4: e quando Ts cresce?');
ylabel('\DeltaH [m]'); xlabel('Tempo [s]'); legend('Location','southeast'); grid on;

% >>> Por que tau/2 explode, se a malha acomoda em ~222 s e a "regra"
% >>> Tac/30 ~ 7,4 s ate permitiria? Anote sua hipotese. Resposta no Lab 12.

%% ------------------------- funcoes locais ---------------------------------
function v = pega(x, k)
% Valor passado com protecao: antes do inicio, tudo era zero (repouso).
    if k >= 1, v = x(k); else, v = 0; end
end

function [t, y, u, estourou] = malha_discreta(Ts, b, Kp, tau, theta, Kp_as, ...
                                  Kc, Ti, Td, Tf, gamma, tsim, r_amp, das_amp, t_dist)
% Mesma logica do laco da Parte 2, embalada em funcao (b variavel).
    s  = tf('s');
    C  = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
    F  = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);
    Cz = c2d(C, Ts, 'tustin');  [nC,dC] = tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
    Fz = c2d(F, Ts, 'tustin');  [nF,dF] = tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
    a1 = exp(-Ts/tau); b1 = Kp*(1-a1); b1d = Kp_as*(1-a1);
    d  = round(theta/Ts);
    N  = round(tsim/Ts); t = (0:N-1)*Ts;
    r  = r_amp*ones(1,N);
    das = das_amp*(t >= t_dist);
    rf = zeros(1,N); e = zeros(1,N); u = zeros(1,N); y = zeros(1,N);
    estourou = false;
    for k = 1:N
        y(k) = a1*pega(y,k-1) + b1*pega(u,k-1-d) + b1d*pega(das,k-1);
        if abs(y(k)) > 10, estourou = true; y(k:end) = NaN; u(k:end) = NaN; break; end
        rf(k) = -dF(2)*pega(rf,k-1) - dF(3)*pega(rf,k-2) ...
                + nF(1)*r(k) + nF(2)*pega(r,k-1) + nF(3)*pega(r,k-2);
        e(k)  = rf(k) - y(k);
        u(k)  = -dC(2)*pega(u,k-1) - dC(3)*pega(u,k-2) ...
                + nC(1)*e(k) + nC(2)*pega(e,k-1) + nC(3)*pega(e,k-2);
    end
end