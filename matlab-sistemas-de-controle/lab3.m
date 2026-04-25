%% LAB 3 - Tanque não linear
clear; 
clc; 
close all;

%% Parâmetros do sistema
A  = 0.5;       % m^2
Hmax = 1;       % m
Ke = 0.05;      % m^3/s
Ks = 0.015;     % m^3/s
ae0 = 0.6;      % abertura de entrada em regime permanente
as0 = 0.5;      % abertura de saída em regime permanente
g  = 9.8;       % m/s^2

%% Cálculo do regime permanente

H_eq = ((Ke*ae0)/(Ks*as0*sqrt(2*g)))^2;

fprintf('H_eq é: %.4f m\n', H_eq);


%% CASO 1: H0 = 0 
H0 = 0; 
out_caso1 = sim("ModeloLab3.slx");

%% CASO 2: H0 = H_eq (Regime permanente)
H0 = H_eq; 
out_caso2 = sim("ModeloLab3.slx");
valor_final = out_caso2.simout.Data(end);
fprintf('Nível final do tanque no Caso 2 = %.4f m\n', valor_final);

%% Comparação dos dois casos
figure(1);

plot(out_caso1.tout, out_caso1.simout.Data, 'b', 'LineWidth', 2); 
hold on;
plot(out_caso2.tout, out_caso2.simout.Data, 'r--', 'LineWidth', 2);

title('Comparação da Resposta do Tanque Não Linear');
xlabel('Tempo (s)');
ylabel('Nível do Tanque H (m)');
legend('Caso 1: H_0 = 0 (Vazio)', 'Caso 2: H_0 = H_{eq} (Regime)', 'Location', 'best');
grid on;
ylim([0 Hmax+0.1]); 