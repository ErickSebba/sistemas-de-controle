%         0.2402
%---------------------------
%  s^2 + 1.3529 s + 82.9356

%_____________________________

 % -0.001444 s + 0.143
%---------------------------
%  s^2 + 1.23 s + 77.9

%Fit to estimation data: 46.51% (stability enforced) 
% FPE: 0.02038, MSE: 0.02032
%%
% ─────────────────────────────────────────────────────────────
% PARÂMETROS E DEFINIÇÃO DOS SISTEMAS
% ─────────────────────────────────────────────────────────────
Kp   = 2.8968e-03;
zeta = 0.0743;
wn   = 9.1069;

% Função de Transferência Atual H(s)
b0 = Kp * wn^2;
a1 = 2.0 * zeta * wn;
a0 = wn^2;

num = [b0];
den = [1.0, a1, a0];
sys_tf = tf(num, den);

% Função de Transferência Anterior H1(s)
num1 = [-0.001444, 0.143];
den1 = [1.0, 1.23, 77.9];
sys_tf1 = tf(num1, den1);

% ─────────────────────────────────────────────────────────────
% PLOT 1: Resposta ao Degrau (Step Response)
% ─────────────────────────────────────────────────────────────
figure(1);
step(sys_tf, 'b', sys_tf1, 'r--');
grid on;
title('Resposta ao Degrau');
xlabel('Tempo (segundos)');
ylabel('Amplitude');
legend('H(s) ', 'H1(s) ', 'Location', 'best');

% ─────────────────────────────────────────────────────────────
% PLOT 2: Diagrama de Bode (Resposta em Frequência)
% ─────────────────────────────────────────────────────────────
figure(2);
bode(sys_tf, 'b', sys_tf1, 'r--');
grid on;
title('Diagrama de Bode - Comparação');
legend('H(s) ', 'H1(s) ', 'ocation', 'best');
%%
T  = readtable('dados_malha_aberta_20260610_141507.csv');  % excitação aleatória
u  = T.pwm;            % entrada
y  = T.alpha_rad;      % saída
Ts = 0.02;             % 20 ms

data = iddata(y, u, Ts);
data.InputName  = 'PWM';     data.InputUnit  = 'duty';
data.OutputName = 'alpha';   data.OutputUnit = 'rad';
data.TimeUnit   = 'seconds';

data = detrend(data);   % remove a média de u e y (recomendado)

T  = readtable('dados_malha_aberta_20260610_132412.csv');  % excitação aleatória
u  = T.pwm;            % entrada
y  = T.alpha_rad;      % saída
Ts = 0.02;             % 20 ms

data2 = iddata(y, u, Ts);
data2.InputName  = 'PWM';     data.InputUnit  = 'duty';
data2.OutputName = 'alpha';   data.OutputUnit = 'rad';
data2.TimeUnit   = 'seconds';

data2 = detrend(data2);   % remove a média de u e y (recomendado)