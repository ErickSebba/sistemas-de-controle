
clear all; close all; clc;

%% 1 - Inicializacao
A_t = 0.5;  ke = 0.05;  ks = 0.015;  g = 9.8;
ae0 = 0.6;  as0 = 0.5;
H0  = ((ke*ae0)/(ks*as0*sqrt(2*g)))^2;        % 0.8163 m

Kp = 2.721;  tau = 27.21;  theta = 2*tau;

par.Kc = 0.2587;  par.Ti = 53.90;  par.Td = 20.66;
gamma  = 1/8;     par.Tf = gamma*par.Td;
par.b  = 0.5;
par.umin = -0.6;  par.umax = +0.4;
par.aw = false;
par.metodo_D = 'tustin';

dt   = tau/200;
dbuf = round(theta/dt);

%% 2 - Parte 1: dois ritmos com o tanque NL (Ts = tau/10)
Ts  = tau/10;  par.Ts = Ts;
[t1, y1, u1] = simula_2ritmos(Ts, dt, 2500, 0.1, 0, inf, par, ...
                              A_t, ke, ks, g, ae0, as0, H0, dbuf);

% comparacao com o Lab 11 (modelo linear, tudo discreto, mesma sintonia)
[tL, yL, uL] = lab11_linear(Ts, Kp, tau, theta, par, 2500, 0.1);

% CORRIGIDO: extrapolacao com yL(end)/uL(end) em vez de 0, para nao
% "zerar" artificialmente os ultimos pontos de t1 que ficam alem de tL.
yLi = interp1(tL, yL, t1, 'previous', yL(end));
fprintf('Parte 1: max|y_NL - y_Lab11| = %.4f m\n', max(abs(y1 - yLi)));

figure(1);
subplot(2,1,1);
plot(t1, y1, 'LineWidth', 1.5); hold on; stairs(tL, yL, '--', 'LineWidth', 1.2);
title('Lab 12 - Parte 1: tanque NL dois ritmos x Lab 11 linear (Ts = \tau/10)');
ylabel('\DeltaH [m]'); legend('NL dois ritmos','Lab 11 linear','Location','southeast'); grid on;
subplot(2,1,2);
% CORRIGIDO: agora compara u1 (NL) com uL (Lab 11), antes so tinha u1.
plot(t1, u1, 'LineWidth', 1.2); hold on; stairs(tL, uL, '--', 'LineWidth', 1.2);
ylabel('\Deltaa_e'); xlabel('Tempo [s]');
legend('NL dois ritmos','Lab 11 linear','Location','southeast'); grid on;

%% 3 - Parte 2: varredura de Ts - a explosao explicada
par.metodo_D = 'tustin';        % <<< DESAFIO: troque por 'euler' e compare
lista_Ts = [tau/20, tau/10, tau/5, tau/2, tau];

fprintf('\nParte 2 (metodo_D = %s):\n', par.metodo_D);
fprintf('   Ts(s)     N    z_D      estado      Mp(%%)     IAE\n');

resultados = cell(numel(lista_Ts),1);
figure(2);
subplot(2,1,1); hold on; cores = lines(numel(lista_Ts));
for i = 1:numel(lista_Ts)
    Tsi = lista_Ts(i);  par.Ts = Tsi;
    if strcmp(par.metodo_D,'tustin')
        zD = (2*par.Tf - Tsi)/(2*par.Tf + Tsi);
    else
        zD = par.Tf/(par.Tf + Tsi);
    end
    [t2, y2, ~] = simula_2ritmos(Tsi, dt, 2500, 0.1, 0, inf, par, ...
                                 A_t, ke, ks, g, ae0, as0, H0, dbuf);
    resultados{i} = struct('t',t2,'y',y2,'Ts',Tsi);
    if any(isnan(y2))
        fprintf('%8.3f  %4d  %+.3f   INSTAVEL       -        -\n', ...
                Tsi, round(Tsi/dt), zD);
        plot(t2, y2, 'Color', cores(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Ts=\\tau/%g (INST.)', tau/Tsi));
    else
        err = 0.1 - y2;  iae = trapz(t2, abs(err));
        Mp  = 100*max(0, max(y2)-0.1)/0.1;
        fprintf('%8.3f  %4d  %+.3f   estavel    %6.2f   %7.3f\n', ...
                Tsi, round(Tsi/dt), zD, Mp, iae);
        plot(t2, y2, 'Color', cores(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Ts=\\tau/%g', tau/Tsi));
    end
end
% CORRIGIDO: escala automatica aqui (antes o ylim fixo [-0.05 0.3]
% cortava fora tau/2 e tau, escondendo a "explosao" que e o ponto central
% da Parte 2).
title(sprintf('Lab 12 - Parte 2: varredura de Ts, escala completa (termo D: %s)', par.metodo_D));
ylabel('\DeltaH [m]'); legend('Location','northeast'); grid on;

% CORRIGIDO: subplot extra com zoom nos casos bem-comportados, para nao
% perder a comparacao fina entre tau/20, tau/10 e tau/5.
subplot(2,1,2); hold on;
idx_zoom = find(lista_Ts <= tau/5);
for k = idx_zoom
    plot(resultados{k}.t, resultados{k}.y, 'Color', cores(k,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Ts=\\tau/%g', tau/resultados{k}.Ts));
end
ylim([-0.02 0.15]);
title('Zoom: casos bem-comportados (\tau/20, \tau/10, \tau/5)');
ylabel('\DeltaH [m]'); xlabel('Tempo [s]'); legend('Location','southeast'); grid on;

%% 4 - Parte 3: windup - a valvula que travou aberta
par.Ts = tau/10;  par.metodo_D = 'tustin';
casos = {false, 'SEM protecao'; true, 'COM clamping'};
res = cell(2,1);
for j = 1:2
    par.aw = casos{j,1};
    [t3, y3, u3, I3] = simula_2ritmos(par.Ts, dt, 5000, 0, 0.5, [100 700], par, ...
                                      A_t, ke, ks, g, ae0, as0, H0, dbuf);
    res{j} = struct('t',t3,'y',y3,'u',u3,'I',I3);
    i700  = find(t3 >= 700, 1);
    pos   = t3 >= 700;
    pico  = max(y3(pos));
    rail  = find(u3 >= par.umax - 1e-3 & t3 >= 700);
    trail = 0; if ~isempty(rail), trail = t3(rail(end)) - 700; end
    fora  = find(abs(y3) > 0.02 & t3 >= 700);
    trec  = 0; if ~isempty(fora), trec = t3(fora(end)) - 700; end
    fprintf('Parte 3 %-12s: I(700s)=%.3f | pico pos=+%.3f m | u colado %5.0f s | recup. %5.0f s\n', ...
            casos{j,2}, I3(i700), pico, trail, trec);
end

figure(3);
subplot(3,1,1);
plot(res{1}.t, res{1}.y, 'LineWidth', 1.5); hold on;
plot(res{2}.t, res{2}.y, 'LineWidth', 1.5);
title('Lab 12 - Parte 3: windup (valvula de saida travada aberta de 100 a 700 s)');
ylabel('\DeltaH [m]'); legend('sem protecao','com clamping','Location','northeast'); grid on;
subplot(3,1,2);
plot(res{1}.t, res{1}.u, 'LineWidth', 1.2); hold on;
plot(res{2}.t, res{2}.u, 'LineWidth', 1.2);
ylabel('\Deltaa_e (sat \pm)'); grid on;
subplot(3,1,3);
plot(res{1}.t, res{1}.I, 'LineWidth', 1.2); hold on;
plot(res{2}.t, res{2}.I, 'LineWidth', 1.2);
ylabel('Integrador I'); xlabel('Tempo [s]'); grid on;

%% ------------------------- funcoes locais ---------------------------------
function [t, y, u, Ivec] = simula_2ritmos(Ts, dt, t_end, r_amp, das_amp, jan_dist, ...
                                          par, A_t, ke, ks, g, ae0, as0, H0, dbuf)
    N = round(Ts/dt);
    n = round(t_end/dt);
    t = (0:n-1)*dt;
    ubuf = zeros(1, dbuf+1);
    y = zeros(1,n); u = zeros(1,n); Ivec = zeros(1,n);
    est = pid_estados_iniciais();
    H = H0; yk = 0; uk = 0;
    for i = 1:n
        ti = t(i);
        if numel(jan_dist) == 2 && ti >= jan_dist(1) && ti < jan_dist(2)
            d_as = das_amp;
        else
            d_as = 0;
        end
        if mod(i-1, N) == 0
            [uk, est] = pid_discreto(r_amp, yk, est, par);
        end
        ubuf = [uk, ubuf(1:end-1)];
        u_del = ubuf(end);
        ae  = min(max(ae0 + u_del, 0), 1);
        a_s = min(max(as0 + d_as, 0), 1);
        H = H + (ke*ae - ks*a_s*sqrt(max(2*g*H,0)))/A_t * dt;
        yk = H - H0;
        if abs(yk) > 1e3
            y(i:end) = NaN; u(i:end) = NaN; Ivec(i:end) = NaN; return;
        end
        y(i) = yk; u(i) = uk; Ivec(i) = est.I;
    end
end

function [t, y, u] = lab11_linear(Ts, Kp, tau, theta, par, tsim, r_amp)
% Reproducao compacta da malha do Lab 11 (linear, tudo discreto, Tustin).
% CORRIGIDO: agora tambem retorna o vetor u para permitir a comparacao
% do sinal de controle na Parte 1 (antes so retornava t, y).
    s  = tf('s');
    C  = par.Kc*(1 + 1/(par.Ti*s) + par.Td*s/(par.Tf*s+1));
    F  = tf([par.b*par.Ti*par.Tf, par.b*par.Ti+par.Tf, 1], ...
            [par.Ti*par.Td*(1+1/8), par.Ti+par.Tf, 1]);
    Cz = c2d(C,Ts,'tustin'); [nC,dC] = tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
    Fz = c2d(F,Ts,'tustin'); [nF,dF] = tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
    a1 = exp(-Ts/tau); b1 = Kp*(1-a1); d = round(theta/Ts);
    Nk = round(tsim/Ts); t = (0:Nk-1)*Ts; r = r_amp*ones(1,Nk);
    rf = zeros(1,Nk); e = zeros(1,Nk); u = zeros(1,Nk); y = zeros(1,Nk);
    pg = @(x,k) (k>=1)*x(max(k,1));
    for k = 1:Nk
        y(k)  = a1*pg(y,k-1) + b1*pg(u,k-1-d);
        rf(k) = -dF(2)*pg(rf,k-1) - dF(3)*pg(rf,k-2) ...
                + nF(1)*r(k) + nF(2)*pg(r,k-1) + nF(3)*pg(r,k-2);
        e(k)  = rf(k) - y(k);
        u(k)  = -dC(2)*pg(u,k-1) - dC(3)*pg(u,k-2) ...
                + nC(1)*e(k) + nC(2)*pg(e,k-1) + nC(3)*pg(e,k-2);
    end
end

function est = pid_estados_iniciais()
    est.I     = 0;
    est.e_ant = 0;
    est.v_ant = 0;
    est.ud    = 0;
end

function [u, est] = pid_discreto(r, y, est, par)
    e = r - y;
    P = par.Kc*(par.b*r - y);
    v = -y;
    if strcmp(par.metodo_D, 'tustin')
        aD = (2*par.Tf - par.Ts)/(2*par.Tf + par.Ts);
        bD = 2*par.Kc*par.Td/(2*par.Tf + par.Ts);
    else
        aD = par.Tf/(par.Tf + par.Ts);
        bD = par.Kc*par.Td/(par.Tf + par.Ts);
    end
    est.ud = aD*est.ud + bD*(v - est.v_ant);
    I_cand = est.I + (par.Kc*par.Ts/(2*par.Ti))*(e + est.e_ant);
    u_ns   = P + I_cand + est.ud;
    u      = min(max(u_ns, par.umin), par.umax);
    if par.aw && (u_ns ~= u) && (sign(e) == sign(u_ns))
        % CLAMPING: congela o integrador
    else
        est.I = I_cand;
    end
    est.e_ant = e;
    est.v_ant = v;
end