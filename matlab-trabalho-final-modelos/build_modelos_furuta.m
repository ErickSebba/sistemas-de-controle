function build_modelos_furuta()
%% ============================================================================
%  build_modelos_furuta.m
%  Cria os TRES modelos Simulink do Trabalho Final, no padrao do professor
%  (aula_11_pronto.slx): blocos cstblocks/LTI System com objetos do workspace
%  (Cz, Fz, G) + subsistemas.
%
%     modelo_1_linear.slx      -> G(s) da Entrega 2 (linear puro)
%     modelo_2_deadzone.slx    -> G(s) + zona morta [-75,+130] + compensacao inversa
%     modelo_3_naolinear.slx   -> equacoes 2 GDL do TCC (MATLAB Function), calibradas
%
%  Uso:
%     build_modelos_furuta;              % cria os 3 .slx e poe Cz,Fz,G no workspace
%     out = sim('modelo_1_linear');
%     figure; plot(out.tout, out.alpha_deg); grid on
%
%  Requer Control System Toolbox (c2d).
%% ============================================================================

%% ---------- PARAMETROS COMUNS ----------
% Planta linear (Entrega 2)
Kp = 2.8968e-3;  zeta = 0.0743;  wn = 9.1069;
s  = tf('s');
G  = (Kp*wn^2)/(s^2 + 2*zeta*wn*s + wn^2);          %#ok<NASGU>  PWM -> alpha (rad)

% Sintonia adotada (candidato escolhido na Fase 2)
Kc = 57.31;  Ti = 0.1015;  Td = 0.4777;  gamma = 1/8;  Tf = gamma*Td;
b  = 0.7;    c  = 0;       Ts = 0.02;

% Zona morta medida (assimetrica) e saturacao
DZP = 130;  DZN = 75;  Umax = 255;
A_dz = (DZP+DZN)/2;   % 102.5   -> compensacao: u = v + A*sign(v) + B
B_dz = (DZP-DZN)/2;   %  27.5

% Perturbacao regulatoria: PULSO (nao degrau sustentado! ver relatorio)
d_amp = 60;  d_t0 = 2.0;  d_dur = 0.3;

% Parametros do modelo NAO LINEAR (2 GDL, TCC calibrado - Fase 0)
P.mp = 45e-3;  P.Lb = 120e-3;  P.lp = 80e-3;  P.Ib = 0.0015;  P.g = 9.81;
P.J     = 5.1271e-4;    % inercia do pendulo NO PIVO (calibrada; Tabela 2 = 3.84e-4)
P.b_al  = 5.7586e-4;    % atrito do pendulo (calibrado)
P.b_th  = 1.0000e-05;         % atrito do braco  <<< NAO IDENTIFICADO - VER RELATORIO
P.k_tau = 5.0868e-4;    % N.m/PWM (calibrado; ausente no TCC)
assignin('base','P',P);

%% ---------- CONTROLADOR DISCRETO (mesmo nos 3 modelos) ----------
%  DISCRETIZACAO MISTA: Tustin no integral, Euler (backward) no derivativo.
%  Aqui, para o Simulink, C(z) e F(z) sao montados por c2d('tustin') no ramo
%  integral+proporcional e Euler no derivativo. Para manter o padrao do professor
%  (LTI System com Cz/Fz), usamos a forma FT equivalente.
C  = Kc*(1 + 1/(Ti*s) + Td*s/(Tf*s+1));
F  = tf([b*Ti*Tf, b*Ti+Tf, 1], [Ti*Td*(1+gamma), Ti+Tf, 1]);
Cz = c2d(C, Ts, 'tustin');       %#ok<NASGU>
Fz = c2d(F, Ts, 'tustin');       %#ok<NASGU>
assignin('base','Cz',Cz);  assignin('base','Fz',Fz);  assignin('base','G',G);
assignin('base','Ts',Ts);

fprintf('Objetos no workspace: G, Cz, Fz, Ts, P\n');

%% ================= MODELO 1: LINEAR DIRETO =================
m1 = 'modelo_1_linear';
novo(m1);
pid_subsystem(m1, Ts);
add_blk(m1,'simulink/Sources/Step','Ref',30,100,40,30);
set_param([m1 '/Ref'],'Time','0','Before','0','After','0');     % REGULATORIO r=0
set_param([m1 '/PID 2DOF Discreto - FT'],'Position',[110 90 220 160]);
add_blk(m1,'simulink/Discontinuities/Saturation','Sat',260,105,40,30);
set_param([m1 '/Sat'],'UpperLimit',num2str(Umax),'LowerLimit',num2str(-Umax));
pert_pulso(m1, d_amp, d_t0, d_dur, 330, 190);
add_blk(m1,'simulink/Math Operations/Sum','SomaPert',400,110,24,40);
set_param([m1 '/SomaPert'],'Inputs','++');
add_blk(m1,'cstblocks/LTI System','Planta',460,100,90,45);
set_param([m1 '/Planta'],'sys','G');
saida(m1, 580);
add_line(m1,'Ref/1','PID 2DOF Discreto - FT/1','autorouting','on');
add_line(m1,'PID 2DOF Discreto - FT/1','Sat/1','autorouting','on');
add_line(m1,'Sat/1','SomaPert/1','autorouting','on');
add_line(m1,'Pert/1','SomaPert/2','autorouting','on');
add_line(m1,'SomaPert/1','Planta/1','autorouting','on');
add_line(m1,'Planta/1','Rad2Deg/1','autorouting','on');
add_line(m1,'Planta/1','PID 2DOF Discreto - FT/2','autorouting','on');
add_line(m1,'Sat/1','Uout/1','autorouting','on');
fecha(m1);

%% ================= MODELO 2: LINEAR + ZONA MORTA =================
m2 = 'modelo_2_deadzone';
novo(m2);
pid_subsystem(m2, Ts);
add_blk(m2,'simulink/Sources/Step','Ref',30,100,40,30);
set_param([m2 '/Ref'],'Time','0','Before','0','After','0');
set_param([m2 '/PID 2DOF Discreto - FT'],'Position',[110 90 220 160]);
add_blk(m2,'simulink/Discontinuities/Saturation','Sat',260,105,40,30);
set_param([m2 '/Sat'],'UpperLimit',num2str(Umax),'LowerLimit',num2str(-Umax));
% --- compensacao inversa da zona morta:  u = v + A*sign(v) + B ---
comp_deadzone(m2, A_dz, B_dz, 320);
% --- zona morta FISICA do motor ---
add_blk(m2,'simulink/Discontinuities/Dead Zone','DZ motor',470,105,50,30);
set_param([m2 '/DZ motor'],'LowerValue',num2str(-DZN),'UpperValue',num2str(DZP));
pert_pulso(m2, d_amp, d_t0, d_dur, 470, 195);
add_blk(m2,'simulink/Math Operations/Sum','SomaPert',550,110,24,40);
set_param([m2 '/SomaPert'],'Inputs','++');
add_blk(m2,'cstblocks/LTI System','Planta',610,100,90,45);
set_param([m2 '/Planta'],'sys','G');
saida(m2, 730);
add_line(m2,'Ref/1','PID 2DOF Discreto - FT/1','autorouting','on');
add_line(m2,'PID 2DOF Discreto - FT/1','Sat/1','autorouting','on');
add_line(m2,'Sat/1','Comp/1','autorouting','on');
add_line(m2,'Sat/1','Sgn/1','autorouting','on');
add_line(m2,'Sgn/1','GA/1','autorouting','on');
add_line(m2,'GA/1','Comp/2','autorouting','on');
add_line(m2,'Bconst/1','Comp/3','autorouting','on');
add_line(m2,'Comp/1','DZ motor/1','autorouting','on');
add_line(m2,'DZ motor/1','SomaPert/1','autorouting','on');
add_line(m2,'Pert/1','SomaPert/2','autorouting','on');
add_line(m2,'SomaPert/1','Planta/1','autorouting','on');
add_line(m2,'Planta/1','Rad2Deg/1','autorouting','on');
add_line(m2,'Planta/1','PID 2DOF Discreto - FT/2','autorouting','on');
add_line(m2,'Sat/1','Uout/1','autorouting','on');
fecha(m2);

%% ================= MODELO 3: NAO LINEAR (2 GDL) =================
m3 = 'modelo_3_naolinear';
novo(m3);
pid_subsystem(m3, Ts);
add_blk(m3,'simulink/Sources/Step','Ref',30,100,40,30);
set_param([m3 '/Ref'],'Time','0','Before','0','After','0');
set_param([m3 '/PID 2DOF Discreto - FT'],'Position',[110 90 220 160]);
add_blk(m3,'simulink/Discontinuities/Saturation','Sat',260,105,40,30);
set_param([m3 '/Sat'],'UpperLimit',num2str(Umax),'LowerLimit',num2str(-Umax));
comp_deadzone(m3, A_dz, B_dz, 320);
add_blk(m3,'simulink/Discontinuities/Dead Zone','DZ motor',470,105,50,30);
set_param([m3 '/DZ motor'],'LowerValue',num2str(-DZN),'UpperValue',num2str(DZP));
pert_pulso(m3, d_amp, d_t0, d_dur, 470, 195);
add_blk(m3,'simulink/Math Operations/Sum','SomaPert',550,110,24,40);
set_param([m3 '/SomaPert'],'Inputs','++');
% --- planta NAO LINEAR: MATLAB Function + integradores ---
planta_naolinear(m3, 610);
saida(m3, 830);
add_line(m3,'Ref/1','PID 2DOF Discreto - FT/1','autorouting','on');
add_line(m3,'PID 2DOF Discreto - FT/1','Sat/1','autorouting','on');
add_line(m3,'Sat/1','Comp/1','autorouting','on');
add_line(m3,'Sat/1','Sgn/1','autorouting','on');
add_line(m3,'Sgn/1','GA/1','autorouting','on');
add_line(m3,'GA/1','Comp/2','autorouting','on');
add_line(m3,'Bconst/1','Comp/3','autorouting','on');
add_line(m3,'Comp/1','DZ motor/1','autorouting','on');
add_line(m3,'DZ motor/1','SomaPert/1','autorouting','on');
add_line(m3,'Pert/1','SomaPert/2','autorouting','on');
add_line(m3,'SomaPert/1','Planta NL/1','autorouting','on');
add_line(m3,'Planta NL/1','Rad2Deg/1','autorouting','on');
add_line(m3,'Planta NL/1','PID 2DOF Discreto - FT/2','autorouting','on');
add_line(m3,'Sat/1','Uout/1','autorouting','on');
fecha(m3);

fprintf(['\nCriados:\n  modelo_1_linear.slx\n  modelo_2_deadzone.slx\n'...
         '  modelo_3_naolinear.slx\n\nRode:  out = sim(''modelo_1_linear'');\n'...
         '       plot(out.tout, out.alpha_deg); grid on\n']);
end

%% ======================= HELPERS =======================
function novo(m)
    if bdIsLoaded(m), close_system(m,0); end
    new_system(m); open_system(m);
end
function fecha(m)
    set_param(m,'Solver','ode45','StopTime','8','SaveOutput','on','SaveTime','on');
    try, Simulink.BlockDiagram.arrangeSystem(m); catch, end
    save_system(m);
end
function add_blk(m,src,nome,x,y,w,h)
    add_block(src,[m '/' nome],'Position',[x y x+w y+h]);
end

function pid_subsystem(m, Ts)
% Subsistema "PID 2DOF Discreto - FT" — IGUAL ao do professor (aula_11):
%   R -> Fz -> (Soma +-) -> Cz -> U ;  Y entra no somador.
    p = [m '/PID 2DOF Discreto - FT'];
    add_block('built-in/Subsystem', p);
    add_block('built-in/Inport',  [p '/R'], 'Position',[30 60 50 74]);
    add_block('built-in/Inport',  [p '/Y'], 'Position',[30 140 50 154]);
    add_block('cstblocks/LTI System', [p '/Fz'], 'Position',[100 50 170 90]);
    set_param([p '/Fz'],'sys','Fz');
    add_block('built-in/Sum', [p '/Erro Integral'], 'Position',[210 62 234 86]);
    set_param([p '/Erro Integral'],'Inputs','+-');
    add_block('cstblocks/LTI System', [p '/Cz'], 'Position',[270 50 340 90]);
    set_param([p '/Cz'],'sys','Cz');
    add_block('built-in/Outport', [p '/U'], 'Position',[380 62 400 76]);
    add_line(p,'R/1','Fz/1');
    add_line(p,'Fz/1','Erro Integral/1');
    add_line(p,'Y/1','Erro Integral/2','autorouting','on');
    add_line(p,'Erro Integral/1','Cz/1');
    add_line(p,'Cz/1','U/1');
end

function comp_deadzone(m, A, B, x)
% Compensacao inversa da zona morta assimetrica:  u = v + A*sign(v) + B
    add_blk(m,'simulink/Math Operations/Sign','Sgn',x,170,26,26);
    add_blk(m,'simulink/Math Operations/Gain','GA',x+50,170,40,26);
    set_param([m '/GA'],'Gain',num2str(A));
    add_blk(m,'simulink/Sources/Constant','Bconst',x+50,220,40,26);
    set_param([m '/Bconst'],'Value',num2str(B));
    add_blk(m,'simulink/Math Operations/Sum','Comp',x+110,110,24,40);
    set_param([m '/Comp'],'Inputs','+++');
end

function pert_pulso(m, amp, t0, dur, x, y)
% Perturbacao em PULSO (dois degraus). NAO usar degrau sustentado: no modelo
% nao linear ele acelera o braco ate a bifurcacao centrifuga (ver relatorio).
    add_blk(m,'simulink/Sources/Step','PertA',x,y,40,26);
    set_param([m '/PertA'],'Time',num2str(t0),'Before','0','After',num2str(amp));
    add_blk(m,'simulink/Sources/Step','PertB',x,y+40,40,26);
    set_param([m '/PertB'],'Time',num2str(t0+dur),'Before','0','After',num2str(amp));
    add_blk(m,'simulink/Math Operations/Sum','Pert',x+60,y+15,24,30);
    set_param([m '/Pert'],'Inputs','+-');
    add_line(m,'PertA/1','Pert/1','autorouting','on');
    add_line(m,'PertB/1','Pert/2','autorouting','on');
end

function saida(m, x)
    add_blk(m,'simulink/Math Operations/Gain','Rad2Deg',x,110,50,28);
    set_param([m '/Rad2Deg'],'Gain','180/pi');
    add_blk(m,'simulink/Sinks/To Workspace','AlphaOut',x+80,105,70,34);
    set_param([m '/AlphaOut'],'VariableName','alpha_deg','SaveFormat','Timeseries');
    add_blk(m,'simulink/Sinks/Scope','Escopo',x+80,175,40,34);
    add_blk(m,'simulink/Sinks/To Workspace','Uout',x-60,35,60,30);
    set_param([m '/Uout'],'VariableName','u','SaveFormat','Timeseries');
    add_line(m,'Rad2Deg/1','AlphaOut/1','autorouting','on');
    add_line(m,'Rad2Deg/1','Escopo/1','autorouting','on');
end

function planta_naolinear(m, x)
% Planta NAO LINEAR (2 GDL) como subsistema: MATLAB Function + integradores.
    p = [m '/Planta NL'];
    add_block('built-in/Subsystem', p, 'Position',[x 95 x+110 165]);
    add_block('built-in/Inport', [p '/pwm'], 'Position',[30 100 50 114]);
    fcn = [p '/din'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
              'Position',[110 70 230 150]);
% Busca P do workspace UMA VEZ na hora de construir o bloco
    P = evalin('base','P');
    
    codigo = sprintf([...
      'function xdot = din(x, pwm)\n' ...
      '%%#codegen\n' ...
      'k_tau = %g; Ib = %g; mp = %g; Lb = %g; J = %g;\n' ...
      'lp = %g; b_th = %g; b_al = %g; g = %g;\n' ...
      'th_d = x(3); d = x(2); d_d = x(4);\n' ...
      'tau  = k_tau*pwm;\n' ...
      'M11 = Ib + mp*Lb^2 + J*sin(d)^2;\n' ...
      'M12 = -mp*Lb*lp*cos(d);\n' ...           <-- SINAL CORRIGIDO (-)
      'M22 = J;\n' ...
      'C1  = J*sin(2*d)*th_d*d_d + mp*Lb*lp*sin(d)*d_d^2;\n' ... <-- SINAL CORRIGIDO (+)
      'C2  = -0.5*J*sin(2*d)*th_d^2;\n' ...
      'r1  = tau - C1 - b_th*th_d;\n' ...
      'r2  =     - C2 - b_al*d_d - mp*g*lp*sin(d);\n' ...
      'det = M11*M22 - M12^2;\n' ...
      'th_dd = ( M22*r1 - M12*r2)/det;\n' ...
      'd_dd  = (-M12*r1 + M11*r2)/det;\n' ...
      'xdot  = [th_d; d_d; th_dd; d_dd];\n'], ...
      P.k_tau, P.Ib, P.mp, P.Lb, P.J, P.lp, P.b_th, P.b_al, P.g);
    try
        sf = sfroot; blk = sf.find('-isa','Stateflow.EMChart','Path',fcn);
        blk.Script = codigo;
    catch
        warning(['Nao consegui injetar o codigo da MATLAB Function automaticamente. ' ...
                 'Abra o bloco "Planta NL/din" e cole o codigo do relatorio.']);
    end
    add_block('built-in/Integrator', [p '/Int'], 'Position',[280 85 310 135]);
    set_param([p '/Int'],'InitialCondition','[0;0;0;0]');
    add_block('built-in/Selector', [p '/sel_delta'], 'Position',[350 95 390 125]);
    set_param([p '/sel_delta'],'IndexOptions','Index vector (dialog)', ...
              'Indices','2','InputPortWidth','4');
    add_block('built-in/Outport', [p '/delta'], 'Position',[430 103 450 117]);
    add_line(p,'pwm/1','din/2','autorouting','on');
    add_line(p,'Int/1','din/1','autorouting','on');
    add_line(p,'din/1','Int/1','autorouting','on');
    add_line(p,'Int/1','sel_delta/1','autorouting','on');
    add_line(p,'sel_delta/1','delta/1','autorouting','on');
end
