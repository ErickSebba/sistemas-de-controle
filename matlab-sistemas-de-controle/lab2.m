%% Limpar dados
clc;
clear;
close all;

%% Criação de dados
Kp = 5;
tau = 2;
G = tf(Kp, [tau 1]);

%% 1 - Analítica
syms s1 step1_tempo;
Ga = Kp/(tau*s1+1);
step1 = (1/s1)*Ga;
step1_tempo = ilaplace(step1);

figure(1)
fplot(step1_tempo, [0,10], 'LineWidth', 2);
title('Resposta ao Degrau - Analítica');
ylabel('Y');
xlabel('Tempo');
grid on;

%% 2 - Step Function
figure(2)
step(G);
[Y, T] = step(G,10);

U = ones(length(T), 1);
figure(3);
subplot(2,1,1);
plot(T, Y, 'LineWidth', 2);
title('Resposta ao Degrau - Step');
ylabel('Y');
grid on;

subplot(2,1,2);
plot(T, U, 'LineWidth', 2);
ylabel('U (Entrada)');
xlabel('Tempo');
grid on;

S = stepinfo(G)
Tau_aprox = S.SettlingTime/4;

%% 3 - Lsim
t = 0:0.01:10;
u = zeros(length(t),1);
u(1:end) = 1; 

Y_3 = lsim(G,u,t);

figure(4);
subplot(2,1,1);
plot(t, Y_3, 'LineWidth', 2);
title('Resposta ao Degrau - Lsim');
ylabel('Y');
grid on;

subplot(2,1,2);
plot(t, u, 'LineWidth', 2);
ylabel('U (Entrada)');
xlabel('Tempo');
grid on;

%% 4 - Discreta (Discretização)

Ts = 0.5; 

Gd = c2d(G, Ts, 'zoh'); 


t_d = 0:Ts:10;
u_d = ones(length(t_d), 1); 

Y_4 = lsim(Gd, u_d, t_d);

figure(5);
stairs(t_d, Y_4, 'LineWidth', 2); 
hold on;
stairs(t_d, u_d, '--r', 'LineWidth', 1.5);
title('Resposta ao Degrau - Sistema Discreto');
ylabel('Amplitude');
xlabel('Tempo (s)');
legend('Saída Y(k)', 'Entrada U(k)', 'Location', 'best');
grid on;

%% 5 - Simulink

tempo_simulacao = 10;
out = sim('modelo', tempo_simulacao); 
 
figure(6);
plot(out.tout, out.simout, 'LineWidth', 2); 
title('Resposta ao Degrau - Simulink');
ylabel('Y');
xlabel('Tempo (s)');
grid on;