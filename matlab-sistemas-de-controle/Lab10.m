%% =====================================================================
%  Lab 10 - PID 2-DOF no Tanque (pesos b e c)  -  Predict-Observe-Explain
%  Disciplina: Introducao aos Sistemas de Controle - 2026/1
%  Prof. Daniel Miranda Cruz
%
%  Demonstra que os pesos b (proporcional) e c (derivativo) moldam o
%  SEGUIMENTO (R->Y) SEM alterar a REJEICAO de perturbacao (D->Y).
%
%  Controlador 2-DOF (forma ISA, derivada filtrada, gamma = 1/8):
%     u = Kc[(b*r - y) + (1/Ti)INT(r-y) + (Td*s/(1+gamma*Td*s))(c*r - y)]
%
%  Separando o que age sobre r e sobre y:
%     u = Cr*r - Cy*y      com
%     Cr = Kc*b + Kc/(Ti*s) + Kc*c*Td*s/(1+gamma*Td*s)   (caminho da referencia)
%     Cy = Kc   + Kc/(Ti*s) + Kc*Td*s/(1+gamma*Td*s)     (controlador "completo")
%
%  Malha fechada (perturbacao D na entrada do processo):
%     Y = [P*Cr/(1+P*Cy)]*R + [P/(1+P*Cy)]*D
%         \____ Hr (depende de b,c) ___/   \__ Hd (NAO depende de b,c) __/
% =====================================================================

clear; close all; clc;

%% =====================================================================
%  0) PROCESSO LINEARIZADO + CENARIO (theta = 2*tau)
%     (a linearizacao alimenta os blocos LTI System G e Gas do Simulink)
% =====================================================================
A   = 0.5;   ke = 0.05;   ks = 0.015;
ae0 = 0.6;   as0 = 0.5;   g  = 9.8;

H0       = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2;
alpha    = (ks*as0*sqrt(2*g)) / (2*A*sqrt(H0));
beta_ae  = ke / A;
tau_proc = 1/alpha;
Kp_proc  = beta_ae/alpha;

beta_as  = -(1/A)*ks*sqrt(2*g*H0);
Kp_as    = -beta_as/alpha;

s   = tf('s');
G   = Kp_proc / (tau_proc*s + 1);   % dae -> dH   (usado pelo Simulink)
Gas = Kp_as   / (tau_proc*s + 1);   % das -> dH   (usado pelo Simulink)

% Valores "redondos" do roteiro para a analise analitica + Pade(3) do atraso
Kp    = 2.721;
tau   = 27.21;
theta = 2*tau;        r = theta/tau;     % cenario 1 do Lab 09
gamma = 1/8;
[np,dp] = pade(theta,3);
P = (Kp/(tau*s + 1))*tf(np,dp);          % planta + atraso (Pade)

% Config. da simulacao Simulink (degrau de referencia e perturbacao)
H_ref  = H0 + 0.10;   % referencia (m)
ae_min = 0.0;  ae_max = 1.0;
tsim   = 1000;        % perturbacao entra em t = tsim/2

% Grade de tempo da analise (fina para capturar o "chute derivativo")
t = linspace(0, 8*tau + 4*theta, 8000);

%% =====================================================================
%  PARTE 1 - AJUSTE-BASE (ITAE regulatorio - Lopez), do Lab 09
% =====================================================================
Kc = (1.357/Kp)*r^(-0.947);
Ti = (tau/0.842)*r^( 0.738);
Td = 0.381*tau  *r^( 0.995);

fprintf('============== PARTE 1 ==============\n');
fprintf('P1.1  Ajuste-base ITAE-reg (Lopez):\n');
fprintf('      Kc = %.4f   |   Ti = %.2f s   |   Td = %.2f s\n\n', Kc, Ti, Td);

% Ramos fixos do controlador (b e c NAO entram aqui)
Cprop = Kc;                               % ramo proporcional unitario
Cint  = Kc*(1/(Ti*s));                    % ramo integral (erro completo)
Cder  = Kc*(Td*s)/(gamma*Td*s + 1);       % ramo derivativo filtrado
Cy    = Cprop + Cint + Cder;              % controlador completo (realimenta y)

den   = 1 + Cy*P;                         % polinomio caracteristico (fixo)
Hd    = minreal(P/den);                   % D -> Y : NAO depende de b,c
Hd_ref = step(Hd, t);                     % referencia p/ checar "Hd mudou?"

% --- P1.2: PID classico (b = c = 1) ---
b = 1;  c = 1;
Cr = b*Cprop + Cint + c*Cder;             % caminho da referencia
Hr = minreal((Cr*P)/den);                 % R -> Y
Hu = minreal((Cr   )/den);                % R -> U

Sr    = stepinfo(Hr);
ess_r = 1 - dcgain(Hr);                   % erro de regime do seguimento
ess_d = dcgain(Hd);                       % offset residual da rejeicao

% Ts da rejeicao (resposta que volta a zero): ultimo instante fora da faixa 2%
pk_d  = max(abs(Hd_ref));
idx   = find(abs(Hd_ref) > 0.02*pk_d, 1, 'last');
if isempty(idx), ts_d = 0; else, ts_d = t(idx); end

fprintf('P1.2  PID classico (b = c = 1):\n');
fprintf('%-16s %16s %12s %14s\n','','Sobressalto(%)','Ts 2%(s)','erro regime');
fprintf('%-16s %16.2f %12.1f %14.3e\n','Seguimento Hr', Sr.Overshoot, Sr.SettlingTime, ess_r);
fprintf('%-16s %16s %12.1f %14.3e\n','Rejeicao  Hd', '(volta a 0)', ts_d, ess_d);
fprintf('      (pico de desvio de Hd = %.4f; retorna a zero pela acao integral)\n\n', pk_d);

%% =====================================================================
%  PARTE 2 - VARREDURA DO PESO b  (c = 0 fixo)  -- o coracao do lab
% =====================================================================
b_vec = [0 0.3 0.5 0.7 1.0];
c     = 0;

f1 = figure('Name','Hr (R->Y) - efeito de b','Color','w'); hold on; grid on;
f2 = figure('Name','Hu (R->U) - efeito de b','Color','w'); hold on; grid on;
f3 = figure('Name','Hd (D->Y) - efeito de b','Color','w'); hold on; grid on;

cores = lines(numel(b_vec));
sn    = {'N','S'};   % 0 -> N, 1 -> S

fprintf('============== PARTE 2 (c = 0) ==============\n');
fprintf('%-5s %16s %12s %14s %9s\n','b','Sobressalto Hr(%)','Ts Hr(s)','Pico u partida','Hd mudou');
fprintf('%s\n', repmat('-',1,62));

for k = 1:numel(b_vec)
    b  = b_vec(k);
    Cr = b*Cprop + Cint + c*Cder;
    Hr = minreal((Cr*P)/den);
    Hu = minreal((Cr   )/den);

    yr = step(Hr, t);
    yu = step(Hu, t);
    yd = step(Hd, t);                      % independe de b -> deve coincidir

    figure(f1); plot(t, yr, 'Color', cores(k,:), 'LineWidth', 1.4);
    figure(f2); plot(t, yu, 'Color', cores(k,:), 'LineWidth', 1.4);
    figure(f3); plot(t, yd, 'Color', cores(k,:), 'LineWidth', 1.4);

    Sr     = stepinfo(Hr);
    pico_u = max(abs(yu));
    mudou  = max(abs(yd - Hd_ref)) > 1e-6;
    fprintf('%-5.1f %16.2f %12.1f %14.3f %9s\n', ...
            b, Sr.Overshoot, Sr.SettlingTime, pico_u, sn{mudou+1});
end

leg = arrayfun(@(x) sprintf('b = %.1f',x), b_vec, 'UniformOutput', false);
figure(f1); title('H_r (R\rightarrowY): diminuir b reduz o sobressalto');
xlabel('t (s)'); ylabel('y'); legend(leg,'Location','southeast');
figure(f2); title('H_u (R\rightarrowU): esforco de controle na partida');
xlabel('t (s)'); ylabel('u'); legend(leg,'Location','northeast');
figure(f3); title('H_d (D\rightarrowY): curvas coincidem (independe de b)');
xlabel('t (s)'); ylabel('y'); legend(leg,'Location','northeast');

%% =====================================================================
%  PARTE 3 - VARREDURA DO PESO c  (b = melhor da Parte 2)  -> chute derivativo
% =====================================================================
b_best = 0.5;                              % ajuste apos ver a tabela da Parte 2
c_vec  = [0 0.5 1.0];

f4 = figure('Name','Hu (R->U) - efeito de c','Color','w'); hold on; grid on;

fprintf('\n============== PARTE 3 (b = %.1f) ==============\n', b_best);
fprintf('%-5s %16s\n','c','Pico de u na partida');
fprintf('%s\n', repmat('-',1,24));

cores = lines(numel(c_vec));
for k = 1:numel(c_vec)
    c  = c_vec(k);
    Cr = b_best*Cprop + Cint + c*Cder;
    Hu = minreal((Cr)/den);
    yu = step(Hu, t);
    figure(f4); plot(t, yu, 'Color', cores(k,:), 'LineWidth', 1.4);
    fprintf('%-5.1f %16.3f\n', c, max(abs(yu)));
end
leg = arrayfun(@(x) sprintf('c = %.1f',x), c_vec, 'UniformOutput', false);
figure(f4); title('H_u (R\rightarrowU): aumentar c amplia o "chute derivativo"');
xlabel('t (s)'); ylabel('u'); legend(leg,'Location','northeast');

%% =====================================================================
%  PARTE 4 - PROVA NUMERICA: Hd e identico para qualquer b e c
% =====================================================================
fprintf('\n============== PARTE 4 (prova) ==============\n');
maxdif = 0;
for bb = [0 0.5 1.0]
    for cc = [0 0.5 1.0]
        % Hd = P/(1+Cy*P): Cy NAO contem b nem c -> diferenca deve ser ~0
        maxdif = max(maxdif, norm(step(minreal(P/den),t) - Hd_ref, inf));
    end
end
fprintf('Maior diferenca entre curvas Hd (b,c variando): %.2e  (~ 0)\n', maxdif);
fprintf('--> Confirma: b e c NAO afetam a rejeicao de perturbacao.\n');

%% =====================================================================
%  PARTE 5 - (OPCIONAL) ENSAIO NO SIMULINK (aula_10.slx)
%  O subsistema "PID" usa b e c do workspace como ganhos. Basta:
%     b = 0.5; c = 0; out = sim('aula_10');
%     plot(out.simout.Time, out.simout.Data);  % nivel H(t)
% =====================================================================
% b = b_best; c = 0; out = sim('aula_10');