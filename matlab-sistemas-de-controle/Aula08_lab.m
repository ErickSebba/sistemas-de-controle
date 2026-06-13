%% Aula 08 - Lab 08 - Sintonia Manual do Controlador PID
% Tanque Atmosférico em Malha Fechada

clearvars;
close all;
clc;

%% =====================================================================
% PARÂMETROS DO PROCESSO
% ======================================================================

A   = 0.5;
ke  = 0.05;
ks  = 0.015;

ae0 = 0.6;
as0 = 0.5;

g   = 9.8;

%% =====================================================================
% PONTO DE OPERAÇÃO
% ======================================================================

H0 = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2;

%% =====================================================================
% LINEARIZAÇÃO
% ======================================================================

alfa = (ks*as0*sqrt(2*g))/(2*A*sqrt(H0));

beta_ae = ke/A;

beta_as = -(ks*sqrt(2*g*H0))/A;

tau_proc = 1/alfa;

Kp_ae = beta_ae/alfa;

Kp_as = beta_as/alfa;

Kp_proc = Kp_ae;

% Compatibilidade com o modelo Simulink
alpha = alfa;
tau   = tau_proc;

%% =====================================================================
% VARIÁVEIS EXIGIDAS PELO SIMULINK
% ======================================================================

alpha = alfa;

d_ae = 0;
d_as = 0;

%% =====================================================================
% FUNÇÃO DE TRANSFERÊNCIA DO PROCESSO
% ======================================================================

s = tf('s');
% Funções de transferência esperadas pelo Simulink

Gae = Kp_ae/(tau*s + 1);

Gas = Kp_as/(tau*s + 1);

% compatibilidade com o restante do script
G = Gae;
G = Kp_proc/(tau_proc*s + 1);

fprintf('\n');
fprintf('H0  = %.4f m\n',H0);
fprintf('tau = %.2f s\n',tau_proc);
fprintf('Kp  = %.4f\n',Kp_proc);

%% =====================================================================
% CONFIGURAÇÃO DA SIMULAÇÃO
% ======================================================================

H_ref = H0 + 0.10;

H_t0 = H0;

ae_min = 0;
ae_max = 1;

tsim = 500;

%% =====================================================================
% PARÂMETROS DO PID
% ======================================================================

Kc = 1.0;

Ti = 20;

Td = 0;

gamma = 1/8;

%% =====================================================================
% CONTROLADOR CONTÍNUO
% ======================================================================

if isinf(Ti)

    C_cont = Kc;

elseif Td == 0

    C_cont = Kc*(1 + 1/(Ti*s));

else

    C_cont = Kc*( ...
        1 + ...
        1/(Ti*s) + ...
        Td*s/(1 + gamma*Td*s));

end

%% =====================================================================
% ANÁLISE LINEAR
% ======================================================================

Hr = feedback(C_cont*G,1);

fprintf('\n--- Análise de malha fechada ---\n');

S = stepinfo(Hr);

fprintf('Overshoot      = %.2f %%\n',S.Overshoot);
fprintf('Rise Time      = %.2f s\n',S.RiseTime);
fprintf('Settling Time  = %.2f s\n',S.SettlingTime);

%% =====================================================================
% CONFERÊNCIA DAS VARIÁVEIS DO WORKSPACE
% ======================================================================

disp(' ');
disp('Variáveis disponíveis para o Simulink:');

whos alfa alpha beta_ae beta_as ...
     Kp_ae Kp_as ...
     d_ae d_as ...
     ae0 as0 H0 ...
     H_ref Kc Ti Td gamma

%% =====================================================================
% SIMULAÇÃO
% ======================================================================

out = sim('aula_08.slx');

%% =====================================================================
% LEITURA DOS DADOS
% ======================================================================

t = out.simout.time;

dados = out.simout.Data;

h_sim = dados(:,1);

ae_sim = dados(:,2);

%% =====================================================================
% GRÁFICOS
% ======================================================================

figure

subplot(2,1,1)

plot(t,h_sim,'b','LineWidth',2)
hold on

yline(H_ref,'r--','LineWidth',1.5)

grid on

ylabel('h (m)')

title(sprintf('PID: Kc=%.2f  Ti=%g  Td=%.2f',Kc,Ti,Td))

legend('h(t)','Href')

subplot(2,1,2)

plot(t,ae_sim,'k','LineWidth',2)

grid on

ylabel('ae')

xlabel('Tempo (s)')

title('Ação de Controle')


%% ── Tabelas automáticas (P1, P2, P3) ─────────────────────────────────
dRef = 0.10;
mt = @(C) deal(stepinfo(feedback(C*G,1)), abs(1-dcgain(feedback(C*G,1)))*dRef);

fprintf('\n=== P1: P puro ===\n Kc | Ts(s) | OS(%%) | erro(m)\n');
for Kc = [0.5 1.0 2.0 4.0]
    [S,e] = mt(Kc);
    fprintf('%4.1f | %6.1f | %5.2f | %.4f\n', Kc, S.SettlingTime, S.Overshoot, e);
end

fprintf('\n=== P2: PI (Kc=1) ===\n Ti | erro(m) | OS(%%) | Ts(s)\n');
for Ti = [100 50 20 10]
    [S,e] = mt(1*(1+1/(Ti*s)));
    fprintf('%4d | %.4f | %5.2f | %6.1f\n', Ti, e, S.Overshoot, S.SettlingTime);
end

fprintf('\n=== P3: PID (Kc=1, Ti=30, gamma=1/8) ===\n Td | OS(%%) | Ts(s) | u_pico\n');
for Td = [0 2 5 10]
    if Td==0, C = 1*(1+1/(30*s));
    else,     C = 1*(1+1/(30*s)+Td*s/(1+(1/8)*Td*s)); end
    [S,~] = mt(C);
    [u,~] = step(feedback(C,G), 200);
    fprintf('%4d | %5.2f | %6.1f | %.3f\n', Td, S.Overshoot, S.SettlingTime, max(u)*dRef+0.6);
end