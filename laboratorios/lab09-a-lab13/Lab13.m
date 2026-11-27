%% =====================================================================
%  Lab 13 - Estrategias Avancadas no Tanque (Feedforward e Cascata)
%  Introducao aos Sistemas de Controle - Eng. de Computacao - UFMT
%  Prof. Me. Daniel Miranda Cruz - 2026/1
% ---------------------------------------------------------------------
%  Este script:
%    1) Inicializa o tanque ESTENDIDO (atuador dinamico + pressao variavel)
%    2) Calcula as sintonias (FB unico, malha interna e externa da cascata)
%    3) Roda os ensaios via Simulink  (modelo .slx - ver spec)  -> Secao 5
%    4) ALTERNATIVA STANDALONE (sem Simulink): roda AUTOMATICAMENTE os
%       6 cenarios exigidos pelo roteiro (Partes 1-4), calculando as 4
%       metricas da tabela de resultados (desvio max, recuperacao +-2%,
%       IAE, ae max) e gerando os graficos comparativos.  -> Secao 6
%
%  Modelo Simulink esperado: 'Lab13_tanque_avancado.slx'
%  (estrutura de blocos detalhada em Lab13_modelo_Simulink_spec.md)
%
%  CORRECOES/ACRESCIMOS em relacao a versao anterior:
%   - A versao anterior so simulava UM cenario por execucao (MODE/DIST
%     fixos manualmente) e so calculava o desvio maximo. O roteiro POE
%     pede 6 cenarios (Partes 1-4) e 4 metricas cada (tabela da pag. 4),
%     alem de comparacoes explicitas entre eles (Observar/Explicar).
%   - Foi criada a funcao local simulate_lab13(...) que roda o processo
%     nao linear + controlador para um (MODE,DIST) e devolve as 4
%     metricas, alem dos vetores de tempo para plot.
%   - O script agora executa os 6 cenarios em loop, monta uma tabela
%     (results_table) igual a da folha POE, imprime no console e compara
%     com os valores de referencia do professor.
%   - Foram adicionados graficos comparativos (Parte 1 vs Parte 2,
%     Parte 1 vs Parte 3, Parte 1 vs Parte 4) para apoiar as secoes
%     "Observar" e "Explicar" do roteiro.
% =====================================================================
clear all; close all; clc;

%% 1 - Inicializacao do processo (tanque estendido)
A   = 0.5;        % m^2  - area do tanque
ke  = 0.05;       % m^3/s por unidade de abertura (entrada)
ks  = 0.015;      % m^3/s por unidade de abertura (saida)
g   = 9.8;        % m/s^2
ae0 = 0.6;        % abertura de entrada no ponto de operacao
as0 = 0.5;        % abertura de saida   no ponto de operacao
tau_v = 2.0;      % constante de tempo do atuador (valvula de entrada)

% Ponto de operacao (dh/dt = 0, com Prel = 1)
H0  = (ke*ae0/(ks*as0*sqrt(2*g)))^2;     % = 0.8163 m
sqrt2gH0 = sqrt(2*g*H0);                 % = 4.000 (exato)
Qe0 = ke*ae0;                            % = 0.030 m^3/s
Qs0 = ks*as0*sqrt2gH0;                   % = 0.030 m^3/s  (balanceado)

%% 2 - Linearizacao no ponto de operacao
a   = ks*as0*sqrt(2*g)/(2*sqrt(H0));     % dQs/dH no PO
tau = A/a;                               % = 27.21 s
Kp  = ke/a;                              % = 2.721 m/abertura

%% 3 - Ganhos de FEEDFORWARD (derivados do balanco no PO)
Kff_ae   = ks*sqrt2gH0/ke;     % as -> ae       => 1.2000
Kff_flow = ks*sqrt2gH0;        % as -> Qsp      => 0.0600

%% 4 - Sintonias (IMC-PI)
% 4a) Malha UNICA (baseline): planta nivel<-ae,  Kp/(tau s+1)
lam_s = tau/5;
Kcs   = tau/(Kp*lam_s);    Tis = tau;     % Kcs=1.8375  Tis=27.21

% 4b) Malha INTERNA (vazao): planta ke/(tau_v s+1)
lam_i = 0.8;
Kci   = tau_v/(ke*lam_i);  Tii = tau_v;   % Kci=50.0    Tii=2.0

% 4c) Malha EXTERNA (nivel, cascata): planta vazao->nivel, (1/a)/(tau s+1)
K_o   = 1/a;
lam_o = tau/5;
Kco   = tau/(K_o*lam_o);   Tio = tau;     % Kco=0.0919  Tio=27.21

fprintf('H0=%.4f m | tau=%.2f s | Kp=%.3f | Kff(as->ae)=%.3f\n',H0,tau,Kp,Kff_ae);
fprintf('Cascata: interna Kci=%.2f Tii=%.2f | externa Kco=%.4f Tio=%.2f | sep=%.1fx\n',...
        Kci,Tii,Kco,Tio,lam_o/lam_i);

%% 5 - ENSAIOS VIA SIMULINK (opcional / alternativo a Secao 6)
% No modelo, a variavel 'MODE' seleciona a estrutura:
%   MODE = 1 -> FB unico
%   MODE = 2 -> FB + Feedforward (na valvula, Kff_ae)
%   MODE = 3 -> Cascata (nivel->vazao)
%   MODE = 4 -> Cascata + Feedforward (no setpoint de vazao, Kff_flow)
% E 'DIST' seleciona o disturbio:
%   DIST = 1 -> degrau de demanda  (as: 0.5 -> 0.6 em t = t_dist)
%   DIST = 2 -> queda de pressao   (Prel: 1 -> 0.81 em t = t_dist)
%   DIST = 3 -> ambos simultaneos
%
% --- Exemplo de chamada (descomente apos construir o .slx) ------------
% MODE = 3; DIST = 2;
% out = sim('Lab13_tanque_avancado');
% H  = out.H.Data;  t = out.H.Time;  ae = out.ae.Data;  Q = out.Q.Data;
% maxdev = max(abs(H - H0));
% figure; plot(t, H,'LineWidth',1.6); yline(H0,'--'); grid on;
% xlabel('Tempo (s)'); ylabel('Nivel H (m)');
% title(sprintf('MODE=%d DIST=%d | desvio max = %.1f mm',MODE,DIST,maxdev*1000));

%% 6 - ALTERNATIVA STANDALONE (sem Simulink) - roda os 6 cenarios do roteiro
% Simulacao do processo nao linear por passo fino + controladores discretos.
% Espelha a estrutura de firmware (controlador separado do processo).

% Parametros comuns de simulacao
p.A = A; p.ke = ke; p.ks = ks; p.g = g; p.ae0 = ae0; p.as0 = as0;
p.tau_v = tau_v; p.H0 = H0; p.Qe0 = Qe0;
p.Kcs = Kcs; p.Tis = Tis; p.Kci = Kci; p.Tii = Tii; p.Kco = Kco; p.Tio = Tio;
p.Kff_ae = Kff_ae; p.Kff_flow = Kff_flow;
p.tsim   = 400;     % s
p.dt     = 0.01;    % s - passo de integracao do PROCESSO
p.t_dist = 50;      % s  - instante de aplicacao do disturbio
p.as_step  = 0.10;  % degrau de abertura de saida
p.Prel_low = 0.81;  % pressao relativa apos a queda
p.Href     = H0;    % regulatorio: setpoint = ponto de operacao

% Os 6 cenarios exigidos pela "Tabela de resultados" do roteiro (pag. 4)
% MODE: 1=FB unico | 2=FB+FF(valvula) | 3=Cascata | 4=Cascata+FF(vazao)
% DIST: 1=demanda   | 2=pressao        | 3=ambos
cenarios = { ...
    'Demanda - FB puro',       1, 1; ...
    'Demanda - FB+FF',         2, 1; ...
    'Pressao - FB puro',       1, 2; ...
    'Pressao - Cascata',       3, 2; ...
    'Ambos - FB puro',         1, 3; ...
    'Ambos - Cascata+FF',      4, 3  ...
};

nC = size(cenarios,1);
Nome        = strings(nC,1);
DesvioMax_mm  = zeros(nC,1);
Recuperacao_s = zeros(nC,1);
IAE           = zeros(nC,1);
aeMax         = zeros(nC,1);

% Guarda os logs de cada cenario para os graficos comparativos
logs = struct('t',cell(nC,1),'H',cell(nC,1),'Q',cell(nC,1),'ae',cell(nC,1),'u',cell(nC,1));

for i = 1:nC
    MODE = cenarios{i,2};
    DIST = cenarios{i,3};
    [maxdev_mm, trec, iae, aemax, tt, Hlog, Qlog, aelog, ulog] = simulate_lab13(MODE, DIST, p);

    Nome(i)          = cenarios{i,1};
    DesvioMax_mm(i)  = maxdev_mm;
    Recuperacao_s(i) = trec;
    IAE(i)           = iae;
    aeMax(i)          = aemax;

    logs(i).t  = tt;  logs(i).H  = Hlog;
    logs(i).Q  = Qlog; logs(i).ae = aelog; logs(i).u = ulog;

    fprintf('[%d/%d] %-22s MODE=%d DIST=%d -> desvio=%.2f mm | rec=%.1f s | IAE=%.4f | ae_max=%.3f\n', ...
        i, nC, cenarios{i,1}, MODE, DIST, maxdev_mm, trec, iae, aemax);
end

% Monta e exibe a tabela de resultados (mesmo layout da folha POE)
results_table = table(Nome, DesvioMax_mm, Recuperacao_s, IAE, aeMax, ...
    'VariableNames', {'Cenario_Estrutura','DesvioMax_mm','Recuperacao_2pct_s','IAE','ae_max'});
disp(' ');
disp('===== Tabela de resultados (Secao "Tabela de resultados (preencher)") =====');
disp(results_table);

% Comparacao com os valores de referencia do professor (cenarios nominais)
ref_mm = [49; 15; 27; 4; 79; 12];
disp(' ');
disp('===== Comparacao com valores de referencia do professor =====');
comparacao = table(Nome, DesvioMax_mm, ref_mm, DesvioMax_mm - ref_mm, ...
    'VariableNames', {'Cenario_Estrutura','Simulado_mm','Referencia_mm','Diferenca_mm'});
disp(comparacao);

%% 7 - GRAFICOS COMPARATIVOS (apoiam as secoes Observar/Explicar)

% Parte 1: FB puro nos dois disturbios (P1.1-P1.3, E1.1-E1.2)
figure('Name','Parte 1 - Baseline FB puro');
subplot(2,1,1);
plot(logs(1).t, logs(1).H*1000,'LineWidth',1.6); yline(H0*1000,'--k'); grid on;
ylabel('H (mm)'); title(sprintf('%s | desvio max = %.1f mm', Nome(1), DesvioMax_mm(1)));
subplot(2,1,2);
plot(logs(3).t, logs(3).H*1000,'LineWidth',1.6); yline(H0*1000,'--k'); grid on;
ylabel('H (mm)'); xlabel('Tempo (s)');
title(sprintf('%s | desvio max = %.1f mm', Nome(3), DesvioMax_mm(3)));

% Parte 2: demanda, FB puro vs FB+FF (P2.1-P2.3, E2.1-E2.3)
figure('Name','Parte 2 - Feedforward na demanda');
plot(logs(1).t, logs(1).H*1000,'LineWidth',1.4); hold on;
plot(logs(2).t, logs(2).H*1000,'LineWidth',1.6);
yline(H0*1000,'--k'); grid on; legend(Nome(1),Nome(2),'Location','best');
xlabel('Tempo (s)'); ylabel('H (mm)');
title('Demanda: FB puro vs FB+FF');

% Parte 3: pressao, FB puro vs Cascata (P3.1-P3.3, E3.1-E3.3)
figure('Name','Parte 3 - Cascata nivel->vazao');
subplot(2,1,1);
plot(logs(3).t, logs(3).H*1000,'LineWidth',1.4); hold on;
plot(logs(4).t, logs(4).H*1000,'LineWidth',1.6);
yline(H0*1000,'--k'); grid on; legend(Nome(3),Nome(4),'Location','best');
ylabel('H (mm)'); title('Pressao: FB puro vs Cascata');
subplot(2,1,2);
plot(logs(3).t, logs(3).Q,'LineWidth',1.2); hold on;
plot(logs(4).t, logs(4).Q,'LineWidth',1.4);
yline(Qe0,'--k'); grid on; legend(Nome(3),Nome(4),'Location','best');
xlabel('Tempo (s)'); ylabel('Q_e (m^3/s)'); title('Vazao medida (malha interna)');

% Parte 4: ambos os disturbios, FB puro vs Cascata+FF (P4.1-P4.2, E4.1-E4.2)
figure('Name','Parte 4 - Estrutura combinada');
plot(logs(5).t, logs(5).H*1000,'LineWidth',1.4); hold on;
plot(logs(6).t, logs(6).H*1000,'LineWidth',1.6);
yline(H0*1000,'--k'); grid on; legend(Nome(5),Nome(6),'Location','best');
xlabel('Tempo (s)'); ylabel('H (mm)');
title('Ambos os disturbios: FB puro vs Cascata+FF');

% ---------------------------------------------------------------------
% Grafico detalhado de UM cenario (compativel com a versao anterior),
% caso queira inspecionar abertura real vs comando de um caso especifico.
% Troque o indice abaixo (1 a 6) para escolher o cenario.
idx_detalhe = 6;
figure('Name',['Lab13 - detalhe: ' char(Nome(idx_detalhe))]);
subplot(3,1,1);
plot(logs(idx_detalhe).t, logs(idx_detalhe).H,'LineWidth',1.6); yline(H0,'--k'); grid on;
ylabel('H (m)'); title(sprintf('%s | desvio max = %.1f mm', Nome(idx_detalhe), DesvioMax_mm(idx_detalhe)));
subplot(3,1,2);
plot(logs(idx_detalhe).t, logs(idx_detalhe).Q,'LineWidth',1.4); yline(Qe0,'--k'); grid on;
ylabel('Q_e (m^3/s)');
subplot(3,1,3);
plot(logs(idx_detalhe).t, logs(idx_detalhe).ae,'LineWidth',1.4); hold on;
plot(logs(idx_detalhe).t, logs(idx_detalhe).u,':','LineWidth',1.2);
grid on; ylabel('abertura'); xlabel('Tempo (s)');
legend('a_e (real)','u (comando)','Location','best');


%% ======================================================================
%  FUNCOES LOCAIS (devem ficar no final do arquivo de script)
% ======================================================================
function [maxdev_mm, trec, iae, aemax, tt, Hlog, Qlog, aelog, ulog] = simulate_lab13(MODE, DIST, p)
% Roda a simulacao nao linear do tanque estendido para um par (MODE,DIST)
% e devolve as 4 metricas da tabela do roteiro + os vetores de log.
%
%   MODE: 1=FB unico | 2=FB+FF(valvula) | 3=Cascata | 4=Cascata+FF(vazao)
%   DIST: 1=demanda(as) | 2=pressao(Prel) | 3=ambos

    dt = p.dt;
    N  = round(p.tsim/dt);
    tt = (0:N-1)*dt;

    % Perfis de disturbio
    as_t   = p.as0  + (tt>=p.t_dist).*((DIST==1)|(DIST==3))*p.as_step;
    Prel_t = 1      - (tt>=p.t_dist).*((DIST==2)|(DIST==3))*(1-p.Prel_low);

    % Estados
    H  = p.H0; ae = p.ae0;
    I1 = 0; I2 = 0;            % integradores (externo/unico e interno)
    Hlog = zeros(1,N); aelog = Hlog; Qlog = Hlog; ulog = Hlog; eHlog = Hlog;

    for k = 1:N
        % --- PROCESSO (planta nao linear) ---
        Qe = p.ke*ae*sqrt(Prel_t(k));
        Qs = p.ks*as_t(k)*sqrt(2*p.g*max(H,1e-9));
        Q  = Qe;                              % vazao de entrada medida

        % --- CONTROLADOR ---
        switch MODE
            case 1  % FB unico
                eH  = p.Href - H;  I1 = I1 + eH*dt;
                u   = p.ae0 + p.Kcs*(eH + I1/p.Tis);
            case 2  % FB + feedforward (valvula)
                eH  = p.Href - H;  I1 = I1 + eH*dt;
                u   = p.ae0 + p.Kcs*(eH + I1/p.Tis) + p.Kff_ae*(as_t(k)-p.as0);
            case 3  % Cascata
                eH  = p.Href - H;  I1 = I1 + eH*dt;
                Qsp = p.Qe0 + p.Kco*(eH + I1/p.Tio);
                eQ  = Qsp - Q;   I2 = I2 + eQ*dt;
                u   = p.ae0 + p.Kci*(eQ + I2/p.Tii);
            case 4  % Cascata + feedforward (setpoint de vazao)
                eH  = p.Href - H;  I1 = I1 + eH*dt;
                Qsp = p.Qe0 + p.Kco*(eH + I1/p.Tio) + p.Kff_flow*(as_t(k)-p.as0);
                eQ  = Qsp - Q;   I2 = I2 + eQ*dt;
                u   = p.ae0 + p.Kci*(eQ + I2/p.Tii);
        end
        u = min(max(u,0),1);                  % saturacao fisica da valvula

        % --- ATUADOR DINAMICO + INTEGRACAO DO NIVEL (Euler no passo fino) ---
        ae = ae + dt*(u - ae)/p.tau_v;
        H  = H  + dt*(Qe - Qs)/p.A;

        Hlog(k)=H; aelog(k)=ae; Qlog(k)=Q; ulog(k)=u; eHlog(k) = p.Href - H;
    end

    % --- Metricas ---
    maxdev_mm = max(abs(Hlog - p.H0)) * 1000;

    % IAE (integral do erro absoluto) ao longo de toda a simulacao
    iae = trapz(tt, abs(eHlog));

    % ae maximo (excursao real da valvula)
    aemax = max(aelog);

    % Tempo de recuperacao: tempo, apos o disturbio, ate o nivel entrar
    % e permanecer dentro da faixa de +-2% de H0
    band = 0.02 * p.H0;
    idx_dist = find(tt >= p.t_dist, 1, 'first');
    fora_faixa = abs(Hlog - p.H0) > band;
    fora_faixa(1:idx_dist-1) = false;   % ignora antes do disturbio
    ultimo_fora = find(fora_faixa, 1, 'last');
    if isempty(ultimo_fora)
        trec = 0;                        % nunca saiu da faixa (ou retornou no mesmo passo)
    elseif ultimo_fora == N
        trec = NaN;                      % nao recuperou dentro do tempo de simulacao
    else
        trec = tt(ultimo_fora) - p.t_dist;
    end
end