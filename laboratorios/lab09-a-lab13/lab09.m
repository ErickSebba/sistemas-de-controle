%% Aula 09 - Lab 09 - Sintonias Empiricas do PID
%  Disciplina: Introducao aos Sistemas de Controle - 2026/1
%  Prof. Daniel Miranda Cruz
%
%  Processo: tanque atmosferico LINEARIZADO + tempo morto artificial.
%  Compara 8 ajustes (5 metodos; CHR, IAE e ITAE em versao servo e regulatoria).
%
%  Forma do PID (padrao ISA, derivada filtrada):
%     C(s) = Kc*( 1 + 1/(Ti*s) + (Td*s)/(gamma*Td*s + 1) )
%
%  Para o ENSAIO no Simulink (modelo aula_09.slx) o tempo morto deve ser o
%  bloco "Transport Delay" com valor = theta. Este script faz a analise
%  linear equivalente usando aproximacao de Pade para que rode sozinho.

clear all; close all; clc;

%% =====================================================================
%  1) PROCESSO (tanque linearizado dos blocos 1 e 2)
%  =====================================================================
Kp  = 2.721;     % ganho estatico
tau = 27.21;     % constante de tempo (s)

%% =====================================================================
%  2) CENARIO DE TEMPO MORTO  --> ALTERE AQUI (1 ou 2)
%  =====================================================================
cenario = 1;                 % 1: theta = 2*tau   |   2: theta = 5*tau
if cenario == 1
    theta = 2*tau;
else
    theta = 5*tau;
end
r     = theta/tau;           % razao theta/tau (2 ou 5)
gamma = 1/8;                 % filtro derivativo (fixo na disciplina)

fprintf('===== CENARIO %d =====\n', cenario);
fprintf('Kp = %.3f | tau = %.2f s | theta = %.2f s | theta/tau = %.1f\n\n', ...
        Kp, tau, theta, r);

%% =====================================================================
%  3) TABELA DE SINTONIAS (PID, forma ISA)
%  =====================================================================
M = struct('nome',{},'tipo',{},'Kc',{},'Ti',{},'Td',{});

% 1) Ziegler-Nichols malha aberta (decaimento 1/4) -- tabela unica
M(end+1) = struct('nome','ZN-MA','tipo','reg', ...
    'Kc', 1.2*tau/(Kp*theta), 'Ti', 2*theta, 'Td', 0.5*theta);

% 2) CHR 0% sobrevalor -- REGULATORIO
M(end+1) = struct('nome','CHR-reg','tipo','reg', ...
    'Kc', 0.95*tau/(Kp*theta), 'Ti', 2.4*theta, 'Td', 0.42*theta);

% 3) CHR 0% sobrevalor -- SERVO
M(end+1) = struct('nome','CHR-servo','tipo','servo', ...
    'Kc', 0.6*tau/(Kp*theta), 'Ti', tau, 'Td', 0.5*theta);

% 4) Cohen-Coon (decaimento 1/4) -- tabela unica
M(end+1) = struct('nome','Cohen-Coon','tipo','reg', ...
    'Kc', (1/Kp)*(tau/theta)*(4/3 + theta/(4*tau)), ...
    'Ti', theta*(32 + 6*theta/tau)/(13 + 8*theta/tau), ...
    'Td', 4*theta/(11 + 2*theta/tau));

% 5) IAE - Lopez -- REGULATORIO
M(end+1) = struct('nome','IAE-reg','tipo','reg', ...
    'Kc', (1.435/Kp)*r^(-0.921), ...
    'Ti', (tau/0.878)*r^(0.749), ...
    'Td', 0.482*tau*r^(1.137));

% 6) IAE - Rovira -- SERVO
M(end+1) = struct('nome','IAE-servo','tipo','servo', ...
    'Kc', (1.086/Kp)*r^(-0.869), ...
    'Ti', tau/(0.740 - 0.130*r), ...
    'Td', 0.348*tau*r^(0.914));

% 7) ITAE - Lopez -- REGULATORIO
M(end+1) = struct('nome','ITAE-reg','tipo','reg', ...
    'Kc', (1.357/Kp)*r^(-0.947), ...
    'Ti', (tau/0.842)*r^(0.738), ...
    'Td', 0.381*tau*r^(0.995));

% 8) ITAE - Rovira -- SERVO
M(end+1) = struct('nome','ITAE-servo','tipo','servo', ...
    'Kc', (0.965/Kp)*r^(-0.855), ...
    'Ti', tau/(0.796 - 0.147*r), ...
    'Td', 0.308*tau*r^(0.929));

n = numel(M);

%% =====================================================================
%  4) PROCESSO COM TEMPO MORTO (Pade p/ analise linear)
%     OBS: no Simulink use o bloco Transport Delay = theta
%  =====================================================================
s = tf('s');
[np, dp] = pade(theta, 3);        % aproximacao de Pade ordem 3
Pdelay   = tf(np, dp);
P        = (Kp/(tau*s + 1)) * Pdelay;

%% =====================================================================
%  5) MONTAGEM DOS CONTROLADORES E FUNCOES DE MALHA
%  =====================================================================
tfinal = 12*tau + 6*theta;        % horizonte de simulacao adaptativo
t      = linspace(0, tfinal, 4000);

fprintf('%-12s %-6s %10s %10s %10s   %s\n', ...
        'Metodo','Tipo','Kc','Ti(s)','Td(s)','Malha');
fprintf('%s\n', repmat('-',1,66));

cores = lines(n);
figure('Name','Hr (R->Y) - Seguimento'); hold on; grid on;
figure('Name','Hu (R->U) - Esforco de controle'); hold on; grid on;
figure('Name','Hd (D->Y) - Rejeicao de perturbacao'); hold on; grid on;

for i = 1:n
    Kc = M(i).Kc; Ti = M(i).Ti; Td = M(i).Td;

    % Controlador PID ISA com derivada filtrada
    C = Kc*( 1 + 1/(Ti*s) + (Td*s)/(gamma*Td*s + 1) );

    L  = C*P;
    Hr = feedback(L, 1);     % R -> Y
    Hu = feedback(C, P);     % R -> U
    Hd = feedback(P, C);     % D(entrada) -> Y

    est = isstable(Hr);
    if est, status = 'OK'; else, status = 'INSTAVEL'; end

    fprintf('%-12s %-6s %10.4f %10.2f %10.2f   %s\n', ...
            M(i).nome, M(i).tipo, Kc, Ti, Td, status);

    % Plotagens (apenas malhas estaveis para nao explodir a escala)
    if est
        yr = step(Hr, t);
        yu = step(Hu, t);
        yd = step(Hd, t);
        figure(1); plot(t, yr, 'Color', cores(i,:), 'LineWidth', 1.4);
        figure(2); plot(t, yu, 'Color', cores(i,:), 'LineWidth', 1.4);
        figure(3); plot(t, yd, 'Color', cores(i,:), 'LineWidth', 1.4);
    end
end

% Legendas (apenas dos metodos estaveis)
nomes_est = {};
for i = 1:n
    Kc=M(i).Kc; Ti=M(i).Ti; Td=M(i).Td;
    C = Kc*( 1 + 1/(Ti*s) + (Td*s)/(gamma*Td*s + 1) );
    if isstable(feedback(C*P,1)), nomes_est{end+1} = M(i).nome; end %#ok<SAGROW>
end
figure(1); title(sprintf('H_r (R->Y) - Cenario %d (\\theta/\\tau=%.0f)',cenario,r));
xlabel('t (s)'); ylabel('y'); legend(nomes_est,'Location','southeast');
figure(2); title(sprintf('H_u (R->U) - Cenario %d',cenario));
xlabel('t (s)'); ylabel('u'); legend(nomes_est,'Location','northeast');
figure(3); title(sprintf('H_d (D->Y) - Cenario %d',cenario));
xlabel('t (s)'); ylabel('y'); legend(nomes_est,'Location','northeast');

%% =====================================================================
%  6) METRICAS (stepinfo) - tabela de desempenho do seguimento Hr
%  =====================================================================
fprintf('\n--- Desempenho de seguimento (Hr) ---\n');
fprintf('%-12s %12s %14s %12s\n','Metodo','Overshoot(%)','SettlingT(s)','PicoU');
fprintf('%s\n', repmat('-',1,52));
for i = 1:n
    Kc=M(i).Kc; Ti=M(i).Ti; Td=M(i).Td;
    C = Kc*( 1 + 1/(Ti*s) + (Td*s)/(gamma*Td*s + 1) );
    Hr = feedback(C*P,1); Hu = feedback(C,P);
    if isstable(Hr)
        S  = stepinfo(Hr);
        pu = max(abs(step(Hu,t)));
        fprintf('%-12s %12.2f %14.1f %12.3f\n', ...
                M(i).nome, S.Overshoot, S.SettlingTime, pu);
    else
        fprintf('%-12s %12s %14s %12s\n', M(i).nome,'--','--','--');
    end
end

%% =====================================================================
%  7) (OPCIONAL) ENSAIO NO SIMULINK
%  =====================================================================
%  Modelo aula_09.slx esperado:
%    - Entrada degrau de referencia R = H0 + 0.10
%    - Entrada degrau de perturbacao D (na entrada do processo)
%    - Bloco PID com filtro: parametros Kc, Ti, Td, gamma do workspace
%    - Bloco "Transport Delay" = theta  (NAO usar Pade aqui)
%    - Bloco do processo: Kp/(tau*s+1) ou modelo NL do tanque
%    - To Workspace: out.y, out.u
%
%  Para varrer um ajuste especifico no Simulink, escolha o indice:
    idx = 7;  Kc = M(idx).Kc; Ti = M(idx).Ti; Td = M(idx).Td;
    out = sim('aula_09');
    figure; plot(out.tout, out.y); grid on;
    figure;
    subplot(2,1,1);
    plot(out.tout, out.y); grid on;
    xlabel('t (s)'); ylabel('y(t)');
    title(sprintf('Resposta - %s (cenário %d)', M(idx).nome, cenario));

    subplot(2,1,2);
    plot(out.tout, out.u); grid on;
    xlabel('t (s)'); ylabel('u(t)');
    title('Ação de controle');