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
