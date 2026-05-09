%% Limpar dados
clc;
clear;
close all;

%%
A = 0.5;
Ke = 0.05;
Ks = 0.015;
g = 9.8;

% Vetores
ae = 0:0.01:1;
as = 0:0.01:1; % evita divisão por zero

%% 🔹 1. PLOT3 (curva 3D)
H = ((Ke .* ae) ./ (Ks .* as .* sqrt(2*g))).^2;

figure
plot3(as, ae, H, 'b')
xlabel('as')
ylabel('ae')
zlabel('H')
title('Plot3 - Curva 3D')
grid on
axis([0 1 0 1 0 1])

%% 🔹 2. SURF (superfície)
[AE, AS] = meshgrid(ae, as);

H = ((Ke .* AE) ./ (Ks .* AS .* sqrt(2*g))).^2;

figure
surf(AE, AS, H)
xlabel('ae')
ylabel('as')
zlabel('H')
title('Surf - Superfície')
axis([0 1 0 1 0 1])

%% 🔹 3. MESH (grade)
figure
mesh(ae, as, H)
xlabel('ae')
ylabel('as')
zlabel('H')
title('Mesh - Grade 3D')
axis([0 1 0 1 0 1])


%% Ensaios modelos dinamico em malha aberta
ae0 = 0.6;
as0=0.5;
H0 = ((Ke .* ae0) ./ (Ks .* as0 .* sqrt(2*g))).^2;

%% Modelo lab 4 Parametros

tsim = 300*8;
delta_Ae = 0.1;
delta_As = 0.1;
