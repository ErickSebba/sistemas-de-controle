%% Lab 05 - Linearizacao do Tanque Atmosferico
% Disciplina: Introducao aos Sistemas de Controle - 2026/1
% Prof. Daniel Miranda Cruz
% UFMT - Eng. de Computacao
%
% Objetivo: linearizar o modelo NL do tanque em torno do ponto
% de operacao (ae0, as0) e comparar a resposta dos 3 modelos
% (NL, Linear em desvio, Funcao de Transferencia).

clear all; close all; clc;

%% Secao 1 - Inicializacao e ponto de operacao
% Parametros fisicos do processo (mesmos da aula 06)
A   = 0.5;       % m^2 - area da base do tanque
ke  = 0.05;      % m^3/s - constante da valvula de entrada
ks  = 0.015;     % m^3/s - constante da valvula de saida
g   = 9.8;       % m/s^2 - aceleracao da gravidade

% Ponto de operacao escolhido
ae0 = 0.6;       % abertura da valvula de entrada em RP
as0 = 0.5;       % abertura da valvula de saida em RP

% Calculo do nivel em regime permanente (dh/dt = 0)
% ke*ae0 = ks*as0*sqrt(2*g*H0) -->  H0 = ((ke*ae0)/(ks*as0*sqrt(2g)))^2
H0 = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2;

fprintf('===== SECAO 1: Ponto de Operacao =====\n');
fprintf('H0 = %.4f m\n', H0);
fprintf('sqrt(2*g*H0) = %.4f m/s\n', sqrt(2*g*H0));
fprintf('Confirme: ke*ae0 = %.4f vs ks*as0*sqrt(2gH0) = %.4f\n\n', ...
    ke*ae0, ks*as0*sqrt(2*g*H0));

%% Secao 2 - Linearizacao analitica (derivadas parciais)
% f(h, ae, as) = (1/A)*[ke*ae - ks*as*sqrt(2*g*h)]
%
% alpha = df/dh   |0 = -(1/A) * ks*as0 * g / sqrt(2*g*H0)
% beta_ae = df/dae|0 =  ke / A
% beta_as = df/das|0 = -(1/A) * ks * sqrt(2*g*H0)

alpha   = -(1/A) * ks * as0 * g / sqrt(2*g*H0);
beta_ae =  ke / A;
beta_as = -(1/A) * ks * sqrt(2*g*H0);

% Constante de tempo e ganhos da FT
tau   = -1/alpha;
Kp_ae = -beta_ae/alpha;
Kp_as = -beta_as/alpha;

fprintf('===== SECAO 2: Parametros Linearizados =====\n');
fprintf('alpha   (polo)       = %.6f\n', alpha);
fprintf('beta_ae (df/dae|0)   = %.4f\n', beta_ae);
fprintf('beta_as (df/das|0)   = %.4f\n', beta_as);
fprintf('tau   = -1/alpha     = %.2f s\n', tau);
fprintf('Kp_ae = -beta_ae/alpha = %+.4f\n', Kp_ae);
fprintf('Kp_as = -beta_as/alpha = %+.4f\n\n', Kp_as);

%% Secao 3 - Funcao de Transferencia
% Em variaveis de desvio (Dh, Dae, Das):
% dDh/dt = alpha*Dh + beta_ae*Dae + beta_as*Das
% Em Laplace:  Dh(s) = beta_ae/(s-alpha)*Dae(s) + beta_as/(s-alpha)*Das(s)
% Forma canonica de 1a ordem: G(s) = Kp / (tau*s + 1)

s = tf('s');
G_ae = Kp_ae / (tau*s + 1);     % FT da abertura de entrada para o nivel
G_as = Kp_as / (tau*s + 1);     % FT da abertura de saida para o nivel

fprintf('===== SECAO 3: Funcoes de Transferencia =====\n');
disp('G_ae(s) ='); G_ae
disp('G_as(s) ='); G_as

%% Secao 4 - Comparacao NL x Linearizado (degrau pequeno: +5%)
% Sequencia: ae=0.6 (RP) ate t=20s, depois ae=0.63 (degrau de +5%)
tsim = 200;
dt   = 0.1;
t    = (0:dt:tsim)';

% Vetor de entrada para Dae (variavel em desvio)
ae_t = ae0*ones(size(t));
ae_t(t >= 20) = 0.63;      % +5% sobre ae0
Dae  = ae_t - ae0;
as_t = as0*ones(size(t));  % as constante
Das  = as_t - as0;

% --- Simulacao do MODELO NAO-LINEAR via ode45 ---
% dh/dt = (1/A)*[ke*ae(t) - ks*as(t)*sqrt(2*g*h)]
ae_fcn = @(tt) interp1(t, ae_t, tt, 'previous', 'extrap');
as_fcn = @(tt) interp1(t, as_t, tt, 'previous', 'extrap');
odefun = @(tt, h) (1/A)*( ke*ae_fcn(tt) - ks*as_fcn(tt)*sqrt(2*g*max(h,0)) );
[t_nl, h_nl] = ode45(odefun, t, H0);

% --- Simulacao do MODELO LINEARIZADO via lsim ---
% Usa as variaveis em desvio Dae e Das, depois soma H0 para plotar
sys_lin = ss(alpha, [beta_ae, beta_as], 1, 0);   % SS de 1a ordem
Dh_lin  = lsim(sys_lin, [Dae, Das], t);
h_lin   = Dh_lin + H0;                           % volta ao referencial original

% --- Simulacao da FT (mesma coisa, via FT) ---
Dh_ft = lsim(G_ae, Dae, t);   % so Dae varia neste ensaio
h_ft  = Dh_ft + H0;

% --- Plotagem ---
figure(1); clf;
subplot(2,1,1);
plot(t, h_nl, 'b-', 'LineWidth', 2); hold on;
plot(t, h_lin, 'r--', 'LineWidth', 1.5);
plot(t, h_ft,  'g:',  'LineWidth', 1.5);
ylabel('h(t) [m]');
title('Degrau pequeno: \Delta a_e = +5%');
legend('NL (ode45)','Linearizado (lsim)','FT (lsim)','Location','best');
grid on;
subplot(2,1,2);
plot(t, ae_t, 'k', 'LineWidth', 1.5);
ylabel('a_e(t)');
xlabel('Tempo [s]');
ylim([0.55 0.65]); grid on;

erro_max_5pct = max(abs(h_nl - h_lin));
fprintf('===== SECAO 4: Degrau de +5%% =====\n');
fprintf('Maior |h_NL - h_L| = %.6f m\n\n', erro_max_5pct);

%% Secao 5 - Comparacao NL x Linearizado (degrau grande: +50%)
ae_t2 = ae0*ones(size(t));
ae_t2(t >= 20) = 0.9;       % +50% sobre ae0
Dae2  = ae_t2 - ae0;

% NL
ae_fcn2 = @(tt) interp1(t, ae_t2, tt, 'previous', 'extrap');
odefun2 = @(tt, h) (1/A)*( ke*ae_fcn2(tt) - ks*as_fcn(tt)*sqrt(2*g*max(h,0)) );
[~, h_nl2] = ode45(odefun2, t, H0);

% Linear
Dh_lin2 = lsim(sys_lin, [Dae2, Das], t);
h_lin2  = Dh_lin2 + H0;

% FT
Dh_ft2 = lsim(G_ae, Dae2, t);
h_ft2  = Dh_ft2 + H0;

figure(2); clf;
subplot(2,1,1);
plot(t, h_nl2, 'b-', 'LineWidth', 2); hold on;
plot(t, h_lin2,'r--','LineWidth', 1.5);
plot(t, h_ft2, 'g:', 'LineWidth', 1.5);
ylabel('h(t) [m]');
title('Degrau grande: \Delta a_e = +50%');
legend('NL (ode45)','Linearizado (lsim)','FT (lsim)','Location','best');
grid on;
subplot(2,1,2);
plot(t, ae_t2, 'k', 'LineWidth', 1.5);
ylabel('a_e(t)');
xlabel('Tempo [s]');
ylim([0.55 0.95]); grid on;

erro_max_50pct = max(abs(h_nl2 - h_lin2));
fprintf('===== SECAO 5: Degrau de +50%% =====\n');
fprintf('Maior |h_NL - h_L| = %.6f m\n', erro_max_50pct);
fprintf('Razao do erro (50%%/5%%) = %.1f x\n\n', erro_max_50pct/erro_max_5pct);

%% Secao 6 - Pergunta de sintese: comportamento simetrico?
% Voce vera no DESAFIO OPCIONAL (Parte 4 do roteiro):
%   o erro de subir +X% NAO e igual ao erro de descer -X%, porque
%   sqrt(h) e concavo. A linearizacao subestima h ao subir e superestima
%   ao descer (ou vice-versa, dependendo do termo dominante).

fprintf('===== FIM DO LAB 05 =====\n');
fprintf('Volte ao roteiro e responda as questoes E1-E5 da Parte 3.\n');


%%
%% Lab 06 - Comparacao NL x Linearizado x Funcao de Transferencia
% Disciplina: Introducao aos Sistemas de Controle - 2026/1
% Prof. Daniel Miranda Cruz
%
% Objetivo: aplicar uma sequencia de degraus em ae e em as, simular
% os 3 modelos do tanque e comparar visual e quantitativamente.

%% Secao 1 - Inicializacao (mesmos parametros do Lab 05)
A   = 0.5;
ke  = 0.05;
ks  = 0.015;
g   = 9.8;
ae0 = 0.6;
as0 = 0.5;

H0 = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2;

% Linearizacao
alpha   = -(1/A) * ks * as0 * g / sqrt(2*g*H0);
beta_ae =  ke / A;
beta_as = -(1/A) * ks * sqrt(2*g*H0);
tau   = -1/alpha;
Kp_ae = -beta_ae/alpha;
Kp_as = -beta_as/alpha;

s    = tf('s');
G_ae = Kp_ae / (tau*s + 1);
G_as = Kp_as / (tau*s + 1);

fprintf('===== Parametros =====\n');
fprintf('H0 = %.4f m,  tau = %.2f s\n', H0, tau);
fprintf('Kp_ae = %+.4f,  Kp_as = %+.4f\n\n', Kp_ae, Kp_as);

%% Secao 2 - Construcao dos sinais ae(t) e as(t)
% Sequencia inspirada no Simulink padrao da turma, dividida em 8 fases
% de 300s. Aqui usamos degraus de +/- 0.1 (absolutos) para que a
% linearizacao ainda seja "razoavelmente boa" e o aluno veja a divergencia
% sem que ela seja enorme.
%
%   Fase 1: 0    - 300  -> ae=0.7  (+0.1)         as=0.5
%   Fase 2: 300  - 600  -> ae=0.6  (volta)        as=0.5
%   Fase 3: 600  - 900  -> ae=0.5  (-0.1)         as=0.5
%   Fase 4: 900  - 1200 -> ae=0.6  (volta)        as=0.5
%   Fase 5: 1200 - 1500 -> ae=0.6                  as=0.6  (+0.1)
%   Fase 6: 1500 - 1800 -> ae=0.6                  as=0.5  (volta)
%   Fase 7: 1800 - 2100 -> ae=0.6                  as=0.4  (-0.1)
%   Fase 8: 2100 - 2400 -> ae=0.6                  as=0.5  (volta)

d_ae = 0.1;   % degrau absoluto em ae
d_as = 0.1;   % degrau absoluto em as

tsim = 2400;
dt   = 0.1;
t    = (0:dt:tsim)';

ae_t = ae0*ones(size(t));
as_t = as0*ones(size(t));

ae_t(t <  300)               = ae0 + d_ae;
ae_t(t >= 600  & t < 900)    = ae0 - d_ae;

as_t(t >= 1200 & t < 1500)   = as0 + d_as;
as_t(t >= 1800 & t < 2100)   = as0 - d_as;

% Variaveis em desvio
Dae = ae_t - ae0;
Das = as_t - as0;

% Visualizar entradas
figure(10); clf;
subplot(2,1,1);
plot(t, ae_t, 'LineWidth', 1.5); ylabel('a_e(t)'); grid on;
title('Sequencia de ensaio em malha aberta');
ylim([0.45 0.75]);
subplot(2,1,2);
plot(t, as_t, 'LineWidth', 1.5); ylabel('a_s(t)');
xlabel('Tempo [s]'); grid on;
ylim([0.35 0.65]);

%% Secao 3 - Simulacao dos 3 modelos
fprintf('===== Simulando 3 modelos... =====\n');

% --- Modelo NAO-LINEAR (ode45) ---
ae_fcn = @(tt) interp1(t, ae_t, tt, 'previous', 'extrap');
as_fcn = @(tt) interp1(t, as_t, tt, 'previous', 'extrap');
odefun = @(tt, h) (1/A)*( ke*ae_fcn(tt) - ks*as_fcn(tt)*sqrt(2*g*max(h,0)) );
[~, h_NL] = ode45(odefun, t, H0);

% --- Modelo LINEARIZADO (state-space) ---
sys_lin = ss(alpha, [beta_ae, beta_as], 1, 0);
Dh_L    = lsim(sys_lin, [Dae, Das], t);
h_L     = Dh_L + H0;

% --- Modelo POR FT (superposicao) ---
Dh_FT_ae = lsim(G_ae, Dae, t);
Dh_FT_as = lsim(G_as, Das, t);
Dh_FT    = Dh_FT_ae + Dh_FT_as;
h_FT     = Dh_FT + H0;

%% Secao 4 - Plotagem comparativa
figure(11); clf;
subplot(3,1,1);
plot(t, h_NL, 'b-',  'LineWidth', 2); hold on;
plot(t, h_L,  'r--', 'LineWidth', 1.5);
plot(t, h_FT, 'g:',  'LineWidth', 1.5);
yline(H0, 'k:', 'LineWidth', 0.8);
ylabel('h(t) [m]');
title('Comparacao dos 3 modelos do tanque');
legend('Nao-Linear','Linearizado (SS)','FT (lsim)','H_0','Location','best');
grid on;

subplot(3,1,2);
plot(t, ae_t, 'LineWidth', 1.5); hold on;
plot(t, as_t, 'LineWidth', 1.5);
ylabel('aberturas');
legend('a_e','a_s','Location','best');
grid on;

subplot(3,1,3);
plot(t, abs(h_NL - h_L), 'LineWidth', 1.5); hold on;
plot(t, abs(h_L  - h_FT), 'LineWidth', 1.5);
ylabel('|erro| [m]');
xlabel('Tempo [s]');
legend('|h_{NL} - h_L|','|h_L - h_{FT}|','Location','best');
grid on;

%% Secao 5 - Erro maximo absoluto entre NL e L (responde a questao numerica)
% Para variacoes de +/- 0.1 nas aberturas, esperamos um valor proximo de 0.1323 m.
err_max_global = max(abs(h_NL - h_L));
fprintf('\n===== ERRO MAXIMO GLOBAL =====\n');
fprintf('max(abs(h_NL - h_L)) = %.4f m\n', err_max_global);
fprintf('(Valor esperado: ~0.1323 m. Esse valor responde o Q1-Pratica)\n\n');

%% Secao 6 - Erros por fase (analise mais detalhada)
fases = [
      0    300;   % Fase 1: ae +0.1
    600    900;   % Fase 3: ae -0.1
   1200   1500;   % Fase 5: as +0.1
   1800   2100;   % Fase 7: as -0.1
];
nomes = {'Fase 1 (ae +0.1)', ...
         'Fase 3 (ae -0.1)', ...
         'Fase 5 (as +0.1)', ...
         'Fase 7 (as -0.1)'};

fprintf('===== Erros por fase =====\n');
for k = 1:size(fases,1)
    idx = (t >= fases(k,1)) & (t < fases(k,2));
    err_k = max(abs(h_NL(idx) - h_L(idx)));
    fprintf('  %s : err_max = %.4f m\n', nomes{k}, err_k);
end

%% Secao 7 - Verificacao L vs FT (devem ser identicos)
err_L_FT = max(abs(h_L - h_FT));
fprintf('\n===== Verificacao L vs FT =====\n');
fprintf('max(abs(h_L - h_FT)) = %.2e m  (deve ser ~0, so erro numerico)\n', err_L_FT);

% Plot ampliado
figure(12); clf;
plot(t, h_L, 'r--', 'LineWidth', 1.5); hold on;
plot(t, h_FT, 'g:', 'LineWidth', 1.5);
ylabel('h(t) [m]'); xlabel('Tempo [s]');
title('Linearizado (SS) x FT - devem coincidir');
legend('h_L','h_{FT}','Location','best'); grid on;

fprintf('\n===== FIM DO LAB 06 =====\n');
fprintf('Volte ao roteiro e responda as questoes E1-E5.\n');