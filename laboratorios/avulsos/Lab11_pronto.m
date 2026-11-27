
clear all; close all; clc;

%% 1 - Inicializacao: processo, sintonia oficial e ponderacoes 2-DOF
Kp    = 2.721;        % m/abertura
tau   = 27.21;        % s
theta = 2*tau;        % s  (cenario theta = 2*tau do Lab 09)
Kp_as = -3.2653;      % ganho da perturbacao d_as (valvula de saida), sem atraso

% Sintonia ITAE regulatorio (Lopez) - Lab 09
Kc = 0.2587;  Ti = 53.90;  Td = 20.66;
gamma = 1/8;  Tf = gamma*Td;

% Ponderacoes 2-DOF (Lab 10 -> valores oficiais do curso)
b = 0.5;   c = 0;                % derivada so na medicao

% Controlador e pre-filtro continuos (A19 e A21)
s = tf('s');
C = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
F = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);   % F(0)=1

% Planta com atraso interno EXATO
G = Kp/(tau*s+1);   G.InputDelay = theta;

% --- Variaveis extras para a contraprova Simulink (ponto de operacao) ---
A_tank=0.5; ke=0.05; ks=0.015; ae0=0.6; as0=0.5; gg=9.8;
H0    = ((ke*ae0)/(ks*as0*sqrt(2*gg)))^2;   % nivel de operacao (~0.8165 m)
H_ref = H0 + 0.10;                          % degrau servo de +0,1 m
Gas   = Kp_as/(tau*s+1);                    % canal da perturbacao (sem atraso)

%% 2 - Referencia continua: malha 2-DOF exata  (Hr e Hu no estilo Lab 10)
tsim = 2500;   tc = 0:0.5:tsim;
Hr = F*feedback(C*G, 1);      % R -> Y   (seguimento)  = F*C*G/(1+C*G)
Hu = F*feedback(C  , G);      % R -> U   (esforco)     = F*C/(1+C*G)
yc = step(0.1*Hr, tc);        % saida continua (degrau +0,1 m)
uc = step(0.1*Hu, tc);        % ACAO DE CONTROLE continua (para sobrepor ao u(k))
S  = stepinfo(yc, tc, 0.1, 'SettlingTimeThreshold', 0.02)
Ts_regra = S.SettlingTime/30  % ~7,4 s ... guardem esse numero p/ A25!

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
a1  = exp(-Ts/tau);
b1  = Kp*(1-a1);
b1d = Kp_as*(1-a1);                  % canal da perturbacao (sem atraso)

N  = round(tsim/Ts);   t = (0:N-1)*Ts;
r  = 0.1*ones(1,N);                  % degrau servo
das = zeros(1,N);                    % sem perturbacao na Parte 2

rf = zeros(1,N); e = zeros(1,N); u = zeros(1,N); y = zeros(1,N);
for k = 1:N
    rf1 = pega(rf,k-1); rf2 = pega(rf,k-2);
    r1  = pega(r ,k-1); r2  = pega(r ,k-2);
    e1  = pega(e ,k-1); e2  = pega(e ,k-2);
    u1  = pega(u ,k-1); u2  = pega(u ,k-2);
    y1  = pega(y ,k-1);
    ud  = pega(u ,k-1-d);            % <<< o atraso: uma linha, EXATO
    dd  = pega(das,k-1);

    y(k) = a1*y1 + b1*ud + b1d*dd;          % processo (ZOH exato)
    rf(k) = -denFz(2)*rf1 - denFz(3)*rf2 ...
            + numFz(1)*r(k) + numFz(2)*r1 + numFz(3)*r2;   % F(z) filtra r
    e(k) = rf(k) - y(k);
    u(k) = -denCz(2)*u1 - denCz(3)*u2 ...
           + numCz(1)*e(k) + numCz(2)*e1 + numCz(3)*e2;    % C(z) gera u
end

yc_i = interp1(tc, yc, t);
uc_i = interp1(tc, uc, t);
fprintf('\nParte 2: max|y_cont - y_disc| = %.4f m  (%.2f mm)\n', ...
        max(abs(yc_i - y)), 1000*max(abs(yc_i - y)));

figure(1);
subplot(2,1,1);
plot(tc, yc, 'LineWidth', 2); hold on; stairs(t, y, 'LineWidth', 1.5);
title('Lab 11 - Parte 2: saida continua x discreta (Ts = \tau/10)');
ylabel('\DeltaH [m]'); legend('continuo','discreto','Location','southeast'); grid on;
subplot(2,1,2);
plot(tc, uc, 'LineWidth', 2); hold on; stairs(t, u, 'LineWidth', 1.5);
title('Acao de controle: u(t) continuo x u(k) discreto (escada do ZOH)');
ylabel('\Deltaa_e [abertura]'); xlabel('Tempo [s]');
legend('u(t) continuo','u(k) discreto','Location','northeast'); grid on;

%% 5 - Parte 3: 2-DOF discreto - b mexe onde?
% Servo +0,1 m em t=0 e perturbacao d_as=+0,05 em t=1200 s
[t3, yA, uA] = malha_discreta(Ts, 0.5, Kp, tau, theta, Kp_as, Kc, Ti, Td, Tf, gamma, tsim, 0.1, 0.05, 1200);
[~ , yB, uB] = malha_discreta(Ts, 1.0, Kp, tau, theta, Kp_as, Kc, Ti, Td, Tf, gamma, tsim, 0.1, 0.05, 1200);

figure(2);
subplot(2,1,1);
stairs(t3, yA, 'LineWidth', 1.5); hold on; stairs(t3, yB, 'LineWidth', 1.5);
title('Lab 11 - Parte 3: b = 0,5 x b = 1,0 (servo em t=0; perturbacao em t=1200 s)');
ylabel('\DeltaH [m]'); legend('b = 0,5','b = 1,0','Location','southeast'); grid on;
subplot(2,1,2);
stairs(t3, uA, 'LineWidth', 1.5); hold on; stairs(t3, uB, 'LineWidth', 1.5);
ylabel('\Deltaa_e'); xlabel('Tempo [s]'); legend('b = 0,5','b = 1,0'); grid on;

i_servo = t3 < 1200;
fprintf('\nParte 3: Mp servo b=0,5: %.2f%% | b=1,0: %.2f%%\n', ...
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

%% ------------------------- funcoes locais ---------------------------------
function v = pega(x, k)
    if k >= 1, v = x(k); else, v = 0; end
end

function [t, y, u, estourou] = malha_discreta(Ts, b, Kp, tau, theta, Kp_as, ...
                                  Kc, Ti, Td, Tf, gamma, tsim, r_amp, das_amp, t_dist)
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