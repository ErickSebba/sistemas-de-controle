%% Aula 07 - Lab 07 - Malha Fechada + Controle On/Off
%  Tanque Atmosférico
%
%  Objetivos:
%    1. Fechar o laço com controle on/off no Simulink
%    2. Avaliar o efeito da histerese
%    3. Comparar malha aberta x malha fechada na presença de perturbação
%
%  Modelo Simulink necessário: aula_07.slx
%  Estrutura do modelo:
%    - Bloco Constant "H_ref"   → somador de erro
%    - Bloco "h(t) medido"      → da saída do integrador do tanque
%    - Lógica on/off            → If ou Relay block (Deadband = 2*hyst)
%    - Bloco NL do tanque       → mesmo da Aula 06
%    - To Workspace "simout"    → colunas: [h_MF, ae_MF, h_MA, ae_MA]
%
%  ** Para MA: ae fixo em ae0; para MF: ae vem do controlador on/off **

clear all; close all; clc;

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

%% ── Parâmetros do controle On/Off ─────────────────────────────────────
H_ref  = H0 + 0.10;    % referência: H0 + 10 cm
hyst   = 0.02;         % histerese ± (m)  → altere aqui para 3.2

ae_on  = min(ae0 + 0.20, 1.0);   % abertura na ação "liga"
ae_off = max(ae0 - 0.20, 0.0);   % abertura na ação "desliga"

H_t0   = H0;           % condição inicial do integrador
tsim   = 500;          % tempo de simulação (s)

fprintf('\n--- Parâmetros do controle On/Off ---\n');
fprintf('H_ref   = %.4f m\n', H_ref);
fprintf('hyst    = %.4f m\n', hyst);
fprintf('ae_on   = %.2f\n', ae_on);
fprintf('ae_off  = %.2f\n', ae_off);

%% ── Simulação ──────────────────────────────────────────────────────────
% ATENÇÃO: o modelo Simulink aula_07.slx deve estar na mesma pasta
out = sim('aula_07');

%% ── Plotagem ───────────────────────────────────────────────────────────
t  = out.simout.time;
h_MF  = out.simout.Data(:,1);   % nível em malha fechada
ae_MF = out.simout.Data(:,2);   % ação de controle MF
h_MA  = out.simout.Data(:,3);   % nível em malha aberta (ae=ae0 fixo)
ae_MA = out.simout.Data(:,4);   % ae em malha aberta

figure(1);
subplot(3,1,1);
plot(t, h_MF,  'b', 'LineWidth', 2); hold on;
plot(t, h_MA,  'r--', 'LineWidth', 2);
yline(H_ref,           'k-.',  'LineWidth', 1.5);
yline(H_ref + hyst,    'g:',   'LineWidth', 1);
yline(H_ref - hyst,    'g:',   'LineWidth', 1);
title('Nível do Tanque – On/Off × Malha Aberta');
ylabel('h (m)');
legend('MF On/Off','MA (ae=ae_0)','H_{ref}','±\delta');
grid on;

subplot(3,1,2);
stairs(t, ae_MF, 'b', 'LineWidth', 2); hold on;
plot(t,   ae_MA, 'r--', 'LineWidth', 2);
yline(ae0, 'k:', 'LineWidth', 1);
title('Ação de Controle ae(t)');
ylabel('ae');
legend('MF On/Off','MA');
grid on;

subplot(3,1,3);
plot(t, h_MF - H_ref, 'm', 'LineWidth', 2); hold on;
yline(0, 'k--');
title('Erro em Malha Fechada  e(t) = H_{ref} − h(t)');
ylabel('e (m)'); xlabel('Tempo (s)');
grid on;

%% ── Estudo paramétrico: efeito da histerese ───────────────────────────
% Instrução: comente/descomente os valores abaixo, re-simule e registre
% hyst = 0.005;
% hyst = 0.02;
% hyst = 0.08;
% → Re-execute a partir da seção "Parâmetros do controle On/Off"

%% ── Informações úteis para o relatório ───────────────────────────────
% Frequência de chaveamento (aproximada):
%   f_sw ≈ (número de transições) / (2 * tsim)
%   Cálculo manual: contar as subidas em ae_MF e dividir por tsim

% Amplitude da oscilação em h:
%   A_h = max(h_MF) - min(h_MF)
fprintf('\n--- Métricas do ensaio On/Off ---\n');
fprintf('Amplitude de oscilação em h: %.4f m\n', max(h_MF) - min(h_MF));