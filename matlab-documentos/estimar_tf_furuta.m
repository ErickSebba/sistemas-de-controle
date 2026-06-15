%% Estimacao da FT do pendulo de Furuta (referencia para BAIXO)
%  G(s) = Kp*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)   -> 2a ordem ESTAVEL
%
%  Denominador (zeta, wn) -> resposta livre ("soltando"), descartando a subida.
%  Ganho (Kp)             -> impulsos/degraus via tfest (System Id Toolbox).
%
%  Requer: Signal Processing Toolbox (findpeaks) e System Identification Toolbox (tfest/iddata)

clear; clc; close all;

%% ================= 1) RESPOSTA LIVRE -> zeta, wn =================
% Decremento logaritmico no trecho APOS a soltura (subida descartada)
arqs = {"dados_malha_aberta_soltando2.csv", "dados_malha_aberta_soltando.csv"};
t_soltura = [6.86, 6.55];     % ajuste visual do instante de soltura [s]
zw = zeros(numel(arqs),2);

for k = 1:numel(arqs)
    T = readtable(arqs{k});
    t = T.tempo_ms/1000;  a = T.alpha_rad;
    t = t - t(1);

    % ----- ZERA a fase de subida: mantem so o decaimento livre -----
    m  = t >= t_soltura(k);
    tt = t(m) - t(find(m,1));      % tempo a partir da soltura
    aa = a(m);

    % usa so a regiao linear (pequena amplitude) para o decremento
    lin = abs(aa) <= 0.6;  i0 = find(lin,1,'first');
    tt = tt(i0:end) - tt(i0);  aa = aa(i0:end);

    % picos positivos do decaimento
    [pk, loc] = findpeaks(aa, 'MinPeakProminence', 0.02);
    tp = tt(loc);

    % decremento logaritmico: delta = (1/n) ln(x0/xn)
    n = numel(pk) - 1;
    delta = (1/n) * log(pk(1)/pk(end));
    zeta  = delta / sqrt(4*pi^2 + delta^2);

    % frequencia amortecida pelo periodo medio entre picos
    Td = mean(diff(tp));
    wd = 2*pi/Td;
    wn = wd / sqrt(1 - zeta^2);

    zw(k,:) = [zeta wn];
    fprintf('%s : zeta=%.4f  wn=%.4f rad/s\n', arqs{k}, zeta, wn);
end
zeta = mean(zw(:,1));  wn = mean(zw(:,2));

%% ================= 2) IMPULSOS -> Kp (via tfest) =================
T = readtable("dados_malha_aberta_20260610_132412.csv");
t = T.tempo_ms/1000;  t = t - t(1);
u = T.pwm;  y = T.alpha_rad;
Ts = 0.02;                              % periodo de amostragem (20 ms)
tu = (0:Ts:t(end))';
uu = interp1(t,u,tu);  yy = interp1(t,y,tu);

data = iddata(yy, uu, Ts);              % objeto de identificacao
% 2 polos, 0 zeros (forma do professor). Use init com nossos zeta,wn:
init = idtf([wn^2], [1 2*zeta*wn wn^2]);
G    = tfest(data, init);              % refina o modelo nos dados de E/S
% alternativa direta:  G = tfest(data, 2, 0);

[num, den] = tfdata(G, 'v');
wn_id   = sqrt(den(3));
zeta_id = den(2)/(2*wn_id);
Kp      = num(end)/den(end);           % ganho DC = Kp

fprintf('\n--- tfest ---\n');
fprintf('Kp=%.4e  zeta=%.4f  wn=%.4f  (fit=%.1f%%)\n', Kp, zeta_id, wn_id, G.Report.Fit.FitPercent);

%% ================= 3) MODELO FINAL + SOBREPOSICAO ===============
% Usa zeta,wn da resposta livre (mais confiaveis) e Kp do tfest
Gfinal = tf(Kp*wn^2, [1 2*zeta*wn wn^2]);
fprintf('\n=== MODELO FINAL ===\n'); Gfinal

% (a) sobreposicao na resposta livre
figure;
subplot(2,1,1);
T2 = readtable("dados_malha_aberta_soltando2.csv");
t2 = T2.tempo_ms/1000 - T2.tempo_ms(1)/1000;  a2 = T2.alpha_rad;
m = t2 >= t_soltura(1);  tt = t2(m)-t2(find(m,1)); aa = a2(m);
plot(tt, aa, 'b'); hold on;
yenv = max(aa)*exp(-zeta*wn*tt);
plot(tt,  yenv, 'r--', tt, -yenv, 'r--');
title('Resposta livre: experimental vs envelope do modelo'); grid on; legend('exp','envelope');

% (b) sobreposicao nos impulsos
subplot(2,1,2);
ysim = lsim(Gfinal, uu, tu);
plot(tu, yy, 'b', tu, ysim, 'r--'); xlim([33 41]);
title('Impulso: experimental vs G(s)'); grid on; legend('exp','G(s)');
xlabel('t [s]'); ylabel('\alpha [rad]');
