%% Lab 05 - Linearizacao do Tanque Atmosferico
% Disciplina: Introducao aos Sistemas de Controle - 2026/1
% Prof. Daniel Miranda Cruz — UFMT / Eng. de Computacao
%
% Baseado no codigo anterior do aluno (Lab 04 - visualizacoes 3D)
% e no roteiro oficial do Lab 05.
%
% Estrutura:
%   Secao 0 - Visualizacoes 3D do ponto de operacao (reaproveitado do Lab04)
%   Secao 1 - Ponto de operacao e regime permanente
%   Secao 2 - Linearizacao analitica (alpha, beta_ae, beta_as)
%   Secao 3 - Funcoes de Transferencia G_ae e G_as
%   Secao 4 - Comparacao NL x L: degrau PEQUENO (+5%)
%   Secao 5 - Comparacao NL x L: degrau GRANDE (+50%)
%   Secao 6 - Desafio opcional: simetria +20% vs -20%

clear all; close all; clc;

%% ================================================================
%  SECAO 0 — Parametros fisicos e visualizacoes 3D (Lab 04)
%% ================================================================
A  = 0.5;      % m^2  - area da base do tanque
ke = 0.05;     % m^3/s - constante da valvula de entrada
ks = 0.015;    % m^3/s - constante da valvula de saida
g  = 9.8;      % m/s^2 - aceleracao da gravidade

% Vetores para visualizacao (evita divisao por zero comecando em 0.01)
ae_vec = 0.01:0.01:1;
as_vec = 0.01:0.01:1;

% --- Plot3: curva 3D H0 em regime permanente ---
H_plot3 = ((ke .* ae_vec) ./ (ks .* as_vec .* sqrt(2*g))).^2;
figure(1); clf;
plot3(as_vec, ae_vec, H_plot3, 'b', 'LineWidth', 1.5);
xlabel('a_s'); ylabel('a_e'); zlabel('H_0 [m]');
title('Plot3 - Nivel de regime permanente H_0(a_e, a_s)');
grid on; axis([0 1 0 1 0 max(H_plot3)]);

% --- Surf: superficie H0(ae, as) ---
[AE, AS] = meshgrid(ae_vec, as_vec);
H_surf = ((ke .* AE) ./ (ks .* AS .* sqrt(2*g))).^2;
figure(2); clf;
surf(AE, AS, H_surf); shading interp;
xlabel('a_e'); ylabel('a_s'); zlabel('H_0 [m]');
title('Surf - Superficie H_0(a_e, a_s)');
colorbar;

% --- Mesh: grade 3D ---
figure(3); clf;
mesh(ae_vec, as_vec, H_surf);
xlabel('a_e'); ylabel('a_s'); zlabel('H_0 [m]');
title('Mesh - Grade 3D de H_0');

%% ================================================================
%  SECAO 1 — Ponto de operacao e nivel de regime permanente H0
%% ================================================================
fprintf('\n===== SECAO 1: Ponto de Operacao =====\n');

ae0 = 0.6;   % abertura da valvula de entrada no ponto de operacao
as0 = 0.5;   % abertura da valvula de saida  no ponto de operacao

% Balanco em regime permanente: ke*ae0 = ks*as0*sqrt(2*g*H0)
% Isolando H0:
H0 = ((ke * ae0) / (ks * as0 * sqrt(2*g)))^2;

fprintf('ae0 = %.2f  |  as0 = %.2f\n', ae0, as0);
fprintf('H0  = %.4f m\n', H0);
fprintf('sqrt(2*g*H0) = %.4f m/s\n', sqrt(2*g*H0));
fprintf('Verificacao de balanco:\n');
fprintf('  ke*ae0          = %.5f m^3/s\n', ke*ae0);
fprintf('  ks*as0*sqrt(...)= %.5f m^3/s\n', ks*as0*sqrt(2*g*H0));

%% ================================================================
%  SECAO 2 — Linearizacao analitica (serie de Taylor, 1a ordem)
%% ================================================================
% Modelo NL:  dh/dt = f(h, ae, as) = (1/A)*[ke*ae - ks*as*sqrt(2*g*h)]
%
% Derivadas parciais avaliadas em (H0, ae0, as0):
%   alpha   = df/dh  |0 = -(1/A) * ks*as0 * g / sqrt(2*g*H0)
%   beta_ae = df/dae |0 =  ke / A
%   beta_as = df/das |0 = -(1/A) * ks * sqrt(2*g*H0)
%
% Modelo linearizado em desvio (Dh = h - H0, Dae = ae - ae0, Das = as - as0):
%   d(Dh)/dt = alpha*Dh + beta_ae*Dae + beta_as*Das

alpha   = -(1/A) * ks * as0 * g / sqrt(2*g*H0);
beta_ae =  ke / A;
beta_as = -(1/A) * ks * sqrt(2*g*H0);

% Constante de tempo e ganhos estaticos
tau    = -1 / alpha;          % [s]  polo: s = alpha = -1/tau
Kp_ae  = -beta_ae / alpha;    % ganho estatico para entrada ae
Kp_as  = -beta_as / alpha;    % ganho estatico para entrada as

fprintf('\n===== SECAO 2: Parametros Linearizados =====\n');
fprintf('alpha   (polo)         = %+.6f  (1/s)\n', alpha);
fprintf('beta_ae (df/dae|0)     = %+.4f\n', beta_ae);
fprintf('beta_as (df/das|0)     = %+.4f\n', beta_as);
fprintf('tau   = -1/alpha       = %.4f s\n', tau);
fprintf('Kp_ae = -beta_ae/alpha = %+.4f  (m / unidade de ae)\n', Kp_ae);
fprintf('Kp_as = -beta_as/alpha = %+.4f  (m / unidade de as)\n', Kp_as);
fprintf('\nNOTA: Kp_as negativo pois aumentar a saida DIMINUI o nivel.\n');

%% ================================================================
%  SECAO 3 — Funcoes de Transferencia G_ae(s) e G_as(s)
%% ================================================================
% Em Laplace (condicoes iniciais nulas, variaveis em desvio):
%   Dh(s) = [beta_ae/(s - alpha)]*Dae(s)  +  [beta_as/(s - alpha)]*Das(s)
%
% Forma canonica de 1a ordem:   G(s) = Kp / (tau*s + 1)

s     = tf('s');
G_ae  = Kp_ae / (tau*s + 1);
G_as  = Kp_as / (tau*s + 1);

fprintf('\n===== SECAO 3: Funcoes de Transferencia =====\n');
fprintf('G_ae(s) — entrada ae para nivel h:\n'); disp(G_ae);
fprintf('G_as(s) — entrada as para nivel h:\n'); disp(G_as);

%% ================================================================
%  SECAO 4 — Comparacao NL x Linearizado: degrau PEQUENO (+5%)
%% ================================================================
% Degrau: ae sobe de ae0=0.6 para 0.63 em t=20s; as permanece em as0.

tsim = 200;   % [s] duracao da simulacao
dt   = 0.1;
t    = (0:dt:tsim)';

% Entradas
ae_t4 = ae0 * ones(size(t));
ae_t4(t >= 20) = 0.63;          % +5% de ae0
as_t4 = as0 * ones(size(t));    % as constante

Dae4 = ae_t4 - ae0;
Das4 = as_t4 - as0;

% --- Modelo NAO-LINEAR (ode45) ---
ae_fcn4 = @(tt) interp1(t, ae_t4, tt, 'previous', 'extrap');
as_fcn4 = @(tt) interp1(t, as_t4, tt, 'previous', 'extrap');
odefun4 = @(tt, h) (1/A)*( ke*ae_fcn4(tt) - ks*as_fcn4(tt)*sqrt(2*g*max(h,0)) );
[~, h_NL4] = ode45(odefun4, t, H0);

% --- Modelo LINEARIZADO (lsim em espaco de estados) ---
sys_lin = ss(alpha, [beta_ae, beta_as], 1, 0);
Dh_L4   = lsim(sys_lin, [Dae4, Das4], t);
h_L4    = Dh_L4 + H0;

% --- Modelo por FT (lsim, so Dae varia) ---
Dh_FT4 = lsim(G_ae, Dae4, t);
h_FT4  = Dh_FT4 + H0;

% Erro maximo
err_5pct = max(abs(h_NL4 - h_L4));

fprintf('\n===== SECAO 4: Degrau PEQUENO (+5%%) =====\n');
fprintf('ae: %.2f -> %.2f (Dae = +%.2f)\n', ae0, 0.63, 0.03);
fprintf('max |h_NL - h_L| = %.6f m\n', err_5pct);

figure(4); clf;
subplot(2,1,1);
plot(t, h_NL4, 'b-',  'LineWidth', 2); hold on;
plot(t, h_L4,  'r--', 'LineWidth', 1.5);
plot(t, h_FT4, 'g:',  'LineWidth', 1.5);
yline(H0, 'k:', 'LineWidth', 0.8);
ylabel('h(t) [m]');
title('Degrau pequeno: \Deltaa_e = +5% (ae0=0.6 \rightarrow 0.63)');
legend('NL (ode45)','Linearizado (SS)','FT (lsim)','H_0','Location','best');
grid on;

subplot(2,1,2);
plot(t, ae_t4, 'b-', 'LineWidth', 1.5); hold on;
plot(t, as_t4, 'r-', 'LineWidth', 1.5);
ylabel('Aberturas'); xlabel('Tempo [s]');
legend('a_e','a_s','Location','best');
ylim([0.45 0.75]); grid on;

%% ================================================================
%  SECAO 5 — Comparacao NL x Linearizado: degrau GRANDE (+50%)
%% ================================================================
% Degrau: ae sobe de ae0=0.6 para 0.90 em t=20s (Dae = +0.30 = +50%).

ae_t5 = ae0 * ones(size(t));
ae_t5(t >= 20) = 0.90;          % +50% de ae0
Dae5 = ae_t5 - ae0;

% NL
ae_fcn5 = @(tt) interp1(t, ae_t5, tt, 'previous', 'extrap');
odefun5  = @(tt, h) (1/A)*( ke*ae_fcn5(tt) - ks*as_fcn4(tt)*sqrt(2*g*max(h,0)) );
[~, h_NL5] = ode45(odefun5, t, H0);

% Linearizado
Dh_L5 = lsim(sys_lin, [Dae5, Das4], t);
h_L5  = Dh_L5 + H0;

% FT
Dh_FT5 = lsim(G_ae, Dae5, t);
h_FT5  = Dh_FT5 + H0;

err_50pct = max(abs(h_NL5 - h_L5));

fprintf('\n===== SECAO 5: Degrau GRANDE (+50%%) =====\n');
fprintf('ae: %.2f -> %.2f (Dae = +%.2f)\n', ae0, 0.90, 0.30);
fprintf('max |h_NL - h_L| = %.6f m\n', err_50pct);
fprintf('Razao de erros (50%%/5%%) = %.2f x\n', err_50pct/err_5pct);
fprintf('\nInterpretacao: o erro cresce mais que proporcionalmente ao\n');
fprintf('tamanho do degrau — consistente com truncamento da serie de Taylor.\n');

figure(5); clf;
subplot(3,1,1);
plot(t, h_NL5, 'b-',  'LineWidth', 2); hold on;
plot(t, h_L5,  'r--', 'LineWidth', 1.5);
plot(t, h_FT5, 'g:',  'LineWidth', 1.5);
yline(H0, 'k:', 'LineWidth', 0.8);
ylabel('h(t) [m]');
title('Degrau grande: \Deltaa_e = +50% (ae0=0.6 \rightarrow 0.90)');
legend('NL (ode45)','Linearizado (SS)','FT (lsim)','H_0','Location','best');
grid on;

subplot(3,1,2);
plot(t, ae_t5, 'b-', 'LineWidth', 1.5); hold on;
plot(t, as_t4, 'r-', 'LineWidth', 1.5);
ylabel('Aberturas'); legend('a_e','a_s'); ylim([0.45 1.0]); grid on;

subplot(3,1,3);
plot(t, abs(h_NL5 - h_L5), 'm-', 'LineWidth', 1.5);
ylabel('|h_{NL} - h_L| [m]'); xlabel('Tempo [s]');
title('Erro absoluto: degrau grande');
grid on;

%% ================================================================
%  SECAO 6 — Desafio opcional: assimetria +20% vs -20%
%% ================================================================
% Sequencia de 4 degraus em ae (as permanece em as0):
%   0-60s   ae = 0.60 (operacao)
%   60-120s ae = 0.72 (+20%)
%   120-180s ae = 0.60 (volta)
%   180-240s ae = 0.48 (-20%)

tsim6 = 240; dt6 = 0.1;
t6 = (0:dt6:tsim6)';

ae_t6 = ae0 * ones(size(t6));
ae_t6(t6 >= 60  & t6 < 120) = ae0 * 1.20;  % +20%
ae_t6(t6 >= 180 & t6 < 240) = ae0 * 0.80;  % -20%
as_t6 = as0 * ones(size(t6));

Dae6 = ae_t6 - ae0;
Das6 = as_t6 - as0;

% NL
ae_fcn6 = @(tt) interp1(t6, ae_t6, tt, 'previous', 'extrap');
as_fcn6 = @(tt) interp1(t6, as_t6, tt, 'previous', 'extrap');
odefun6 = @(tt, h) (1/A)*( ke*ae_fcn6(tt) - ks*as_fcn6(tt)*sqrt(2*g*max(h,0)) );
[~, h_NL6] = ode45(odefun6, t6, H0);

% Linearizado
Dh_L6 = lsim(sys_lin, [Dae6, Das6], t6);
h_L6  = Dh_L6 + H0;

% Erros por fase
idx_up   = (t6 >= 60  & t6 < 120);
idx_down = (t6 >= 180 & t6 < 240);
err_up   = max(abs(h_NL6(idx_up)   - h_L6(idx_up)));
err_down = max(abs(h_NL6(idx_down) - h_L6(idx_down)));

fprintf('\n===== SECAO 6: Desafio - Assimetria +20%% vs -20%% =====\n');
fprintf('Erro max (+20%%): %.6f m\n', err_up);
fprintf('Erro max (-20%%): %.6f m\n', err_down);
fprintf('Os erros SAO assimetricos porque sqrt(h) e concava:\n');
fprintf('ao subir, o NL sobe MENOS que o linear preve;\n');
fprintf('ao descer, o NL desce MAIS que o linear preve.\n');

figure(6); clf;
subplot(2,1,1);
plot(t6, h_NL6, 'b-',  'LineWidth', 2); hold on;
plot(t6, h_L6,  'r--', 'LineWidth', 1.5);
yline(H0, 'k:', 'LineWidth', 0.8);
ylabel('h(t) [m]');
title('Desafio: sequencia +20% / -20% em a_e');
legend('NL (ode45)','Linearizado (SS)','H_0','Location','best'); grid on;

subplot(2,1,2);
plot(t6, ae_t6, 'k-', 'LineWidth', 1.5);
ylabel('a_e(t)'); xlabel('Tempo [s]');
ylim([0.40 0.80]); grid on;

%% ================================================================
fprintf('\n===== FIM DO LAB 05 =====\n');
fprintf('Valores para o roteiro:\n');
fprintf('  H0      = %.4f m\n', H0);
fprintf('  alpha   = %.6f\n',   alpha);
fprintf('  beta_ae = %.4f\n',   beta_ae);
fprintf('  beta_as = %.4f\n',   beta_as);
fprintf('  tau     = %.2f s\n', tau);
fprintf('  Kp_ae   = %+.4f\n', Kp_ae);
fprintf('  Kp_as   = %+.4f\n', Kp_as);
fprintf('  Erro degrau  +5%%  : %.6f m\n', err_5pct);
fprintf('  Erro degrau +50%%  : %.6f m\n', err_50pct);
fprintf('Volte ao roteiro e responda as questoes E1-E5.\n');