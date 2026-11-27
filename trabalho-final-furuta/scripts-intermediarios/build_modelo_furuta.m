function build_modelo_furuta()
%% ============================================================================
%  build_modelo_furuta.m
%  Cria programaticamente o modelo Simulink "modelo_furuta.slx" do controle
%  PID 2-DOF DISCRETO (regulatorio) do pendulo de Furuta na posicao inferior.
%
%  Estrutura (mesma logica do aula_11.slx / aula_12.slx):
%    Ref(0) -> F(z) -> (Sum +-) -> C(z) -> Sat[-255,255] -> COMP. INVERSA
%           -> Dead Zone[-75,130] (motor) -> (Sum + perturbacao) -> Planta G(s)
%           -> rad2deg -> alpha; realimenta o Sum. To Workspace: alpha_deg, u.
%
%  Uso:
%     build_modelo_furuta;          % cria o .slx
%     out = sim('modelo_furuta');   % simula
%     figure; plot(out.tout, out.alpha_deg); grid on; ylabel('\alpha (graus)');
%
%  Requer Control System Toolbox (c2d).
%% ============================================================================

mdl = 'modelo_furuta';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

% ---------- PARAMETROS (mesmos do trabalho_final_furuta.m) ----------
Kp=2.8968e-3; zeta=0.0743; wn=9.1069;
Kc=57.31; Ti=0.1015; Td=0.4777; gamma=1/8; Tf=gamma*Td;
b=0.7; c=0; Ts=0.02;                 %#ok<NASGU>  (c=0: derivada so na medicao)
DZP=130; DZN=75; Umax=255;
d_pwm=40; t_dist=2.0;                 % perturbacao regulatoria

% ---------- DISCRETIZACAO (Tustin) ----------
s=tf('s');
C=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
F=tf([b*Ti*Tf, b*Ti+Tf, 1],[Ti*Td*(1+gamma), Ti+Tf, 1]);
Cz=c2d(C,Ts,'tustin'); [nC,dC]=tfdata(Cz,'v'); nC=nC/dC(1); dC=dC/dC(1);
Fz=c2d(F,Ts,'tustin'); [nF,dF]=tfdata(Fz,'v'); nF=nF/dF(1); dF=dF/dF(1);
numG=Kp*wn^2; denG=[1 2*zeta*wn wn^2];
A=(DZP+DZN)/2; B=(DZP-DZN)/2;         % compensacao inversa: v + A*sign(v) + B

ap=@(nome,path,x,y,w,h) add_block(path,[mdl '/' nome], ...
        'Position',[x y x+w y+h]);

% ---------- BLOCOS ----------
ap('Ref','simulink/Sources/Step',30,100,40,30);
set_param([mdl '/Ref'],'Time','0','Before','0','After','0');   % REGULATORIO: r=0
% (para ensaio SERVO: After = deg2rad(5))

ap('Fz','simulink/Discrete/Discrete Transfer Fcn',100,95,70,40);
set_param([mdl '/Fz'],'Numerator',mat2str(nF),'Denominator',mat2str(dF), ...
          'SampleTime',num2str(Ts));

ap('Erro','simulink/Math Operations/Sum',210,100,30,30);
set_param([mdl '/Erro'],'Inputs','+-');

ap('Cz','simulink/Discrete/Discrete Transfer Fcn',270,95,70,40);
set_param([mdl '/Cz'],'Numerator',mat2str(nC),'Denominator',mat2str(dC), ...
          'SampleTime',num2str(Ts));

ap('SatV','simulink/Discontinuities/Saturation',370,100,40,30);
set_param([mdl '/SatV'],'UpperLimit',num2str(Umax),'LowerLimit',num2str(-Umax));

% --- compensacao inversa da deadzone: u = v + A*sign(v) + B ---
ap('Sgn','simulink/Math Operations/Sign',440,160,30,30);
ap('GA','simulink/Math Operations/Gain',490,160,40,30);
set_param([mdl '/GA'],'Gain',num2str(A));
ap('Bconst','simulink/Sources/Constant',490,215,40,30);
set_param([mdl '/Bconst'],'Value',num2str(B));
ap('Comp','simulink/Math Operations/Sum',560,110,30,40);
set_param([mdl '/Comp'],'Inputs','+++');

% --- deadzone do motor (nao-linearidade fisica) ---
ap('DeadMotor','simulink/Discontinuities/Dead Zone',620,105,50,30);
set_param([mdl '/DeadMotor'],'LowerValue',num2str(-DZN),'UpperValue',num2str(DZP));

% --- perturbacao de entrada (regulatorio) ---
ap('Pert','simulink/Sources/Step',620,190,40,30);
set_param([mdl '/Pert'],'Time',num2str(t_dist),'Before','0','After',num2str(d_pwm));
ap('SomaPert','simulink/Math Operations/Sum',700,110,30,40);
set_param([mdl '/SomaPert'],'Inputs','++');

% --- planta contínua G(s) ---
ap('Planta','simulink/Continuous/Transfer Fcn',760,100,80,40);
set_param([mdl '/Planta'],'Numerator',mat2str(numG),'Denominator',mat2str(denG));

% --- saida em graus ---
ap('Rad2Deg','simulink/Math Operations/Gain',870,105,50,30);
set_param([mdl '/Rad2Deg'],'Gain','180/pi');

ap('AlphaOut','simulink/Sinks/To Workspace',950,100,70,40);
set_param([mdl '/AlphaOut'],'VariableName','alpha_deg','SaveFormat','Timeseries');
ap('Uout','simulink/Sinks/To Workspace',620,40,70,40);
set_param([mdl '/Uout'],'VariableName','u','SaveFormat','Timeseries');
ap('Escopo','simulink/Sinks/Scope',950,180,50,40);

% ---------- LIGACOES ----------
add_line(mdl,'Ref/1','Fz/1');
add_line(mdl,'Fz/1','Erro/1');
add_line(mdl,'Erro/1','Cz/1');
add_line(mdl,'Cz/1','SatV/1');
add_line(mdl,'SatV/1','Comp/1');            % v
add_line(mdl,'SatV/1','Sgn/1');             % v -> sign
add_line(mdl,'Sgn/1','GA/1');
add_line(mdl,'GA/1','Comp/2');              % A*sign(v)
add_line(mdl,'Bconst/1','Comp/3');          % B
add_line(mdl,'Comp/1','DeadMotor/1');
add_line(mdl,'DeadMotor/1','SomaPert/1');
add_line(mdl,'Pert/1','SomaPert/2');
add_line(mdl,'SomaPert/1','Planta/1');
add_line(mdl,'Planta/1','Rad2Deg/1');
add_line(mdl,'Rad2Deg/1','AlphaOut/1');
add_line(mdl,'Rad2Deg/1','Escopo/1');
add_line(mdl,'SatV/1','Uout/1');
% realimentacao alpha (rad) para o somador de erro
add_line(mdl,'Planta/1','Erro/2','autorouting','on');

% ---------- CONFIG DE SIMULACAO ----------
set_param(mdl,'Solver','ode45','StopTime','8', ...
              'SaveOutput','on','SaveTime','on');
Simulink.BlockDiagram.arrangeSystem(mdl);   % organiza o layout
save_system(mdl);
fprintf('Modelo "%s.slx" criado. Rode: out = sim(''%s'');\n', mdl, mdl);
end
