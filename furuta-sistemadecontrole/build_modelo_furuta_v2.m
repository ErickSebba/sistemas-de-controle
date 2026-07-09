function build_modelo_furuta()
%% ============================================================================
%  build_modelo_furuta.m  (v2 - padrao de subsistemas, igual ao aula_11)
%  Cria "modelo_furuta.slx": controle PID 2-DOF DISCRETO (regulatorio) do
%  pendulo de Furuta na posicao inferior, na MESMA organizacao do modelo do
%  professor (aula_11_pronto.slx):
%
%    SUBSISTEMA "PID 2DOF Discreto - FT":  R,Y -> Fz -> (Soma +-) -> Cz -> U
%    SUBSISTEMA "Planta Furuta":           dae,dpert -> (Soma) -> G(s) -> dalpha
%    TOPO: Ref(0) -> PID -> Sat -> comp.DZ -> DZ motor -> Planta;
%          perturbacao -> Planta; saida -> graus -> Scope/ToWorkspace; realimenta.
%
%  Uso:  build_modelo_furuta;  out=sim('modelo_furuta');
%        figure; plot(out.tout,out.alpha_deg); grid on; ylabel('\alpha (graus)')
%  Requer Control System Toolbox (c2d).
%
%  OBS: como nao tenho MATLAB aqui, confira ao rodar. Se algum caminho de bloco
%  divergir da sua versao, a estrutura acima permite montar/ajustar a mao a partir
%  do seu aula_11_pronto.slx (trocar a planta e a sintonia, e inserir a zona morta).
%% ============================================================================
mdl='modelo_furuta';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

% ---------- PARAMETROS ----------
Kp=2.8968e-3; zeta=0.0743; wn=9.1069;
Kc=57.31; Ti=0.1015; Td=0.4777; gamma=1/8; Tf=gamma*Td;
b=0.7; c=0; Ts=0.02;                 %#ok<NASGU>
DZP=130; DZN=75; Umax=255; d_pwm=40; t_dist=2.0;
s=tf('s');
C=Kc*(1+1/(Ti*s)+Td*s/(Tf*s+1));
F=tf([b*Ti*Tf,b*Ti+Tf,1],[Ti*Td*(1+gamma),Ti+Tf,1]);
Cz=c2d(C,Ts,'tustin');[nC,dC]=tfdata(Cz,'v');nC=nC/dC(1);dC=dC/dC(1);
Fz=c2d(F,Ts,'tustin');[nF,dF]=tfdata(Fz,'v');nF=nF/dF(1);dF=dF/dF(1);
numG=Kp*wn^2; denG=[1 2*zeta*wn wn^2];
A=(DZP+DZN)/2; B=(DZP-DZN)/2;        % comp. inversa: u = v + A*sign(v) + B

add=@(p,n,x,y,w,h) add_block(p,n,'Position',[x y x+w y+h]);

%% ---------- SUBSISTEMA 1: PID 2DOF Discreto - FT ----------
pid=[mdl '/PID 2DOF Discreto - FT']; add_block('built-in/Subsystem',pid);
add('built-in/Inport', [pid '/R'],  30,60,20,14);
add('built-in/Inport', [pid '/Y'],  30,140,20,14);
add('built-in/DiscreteTransferFcn',[pid '/Fz'],90,50,70,34);
set_param([pid '/Fz'],'Numerator',mat2str(nF),'Denominator',mat2str(dF),'SampleTime',num2str(Ts));
add('built-in/Sum',[pid '/Erro Integral'],200,60,24,24);
set_param([pid '/Erro Integral'],'Inputs','+-');
add('built-in/DiscreteTransferFcn',[pid '/Cz'],270,50,70,34);
set_param([pid '/Cz'],'Numerator',mat2str(nC),'Denominator',mat2str(dC),'SampleTime',num2str(Ts));
add('built-in/Outport',[pid '/U'],380,60,20,14);
add_line(pid,'R/1','Fz/1'); add_line(pid,'Fz/1','Erro Integral/1');
add_line(pid,'Y/1','Erro Integral/2','autorouting','on');
add_line(pid,'Erro Integral/1','Cz/1'); add_line(pid,'Cz/1','U/1');

%% ---------- SUBSISTEMA 2: Planta Furuta ----------
pl=[mdl '/Planta Furuta']; add_block('built-in/Subsystem',pl);
add('built-in/Inport', [pl '/dae'],  30,60,20,14);
add('built-in/Inport', [pl '/dpert'],30,120,20,14);
add('built-in/Sum',[pl '/Soma'],90,70,24,24); set_param([pl '/Soma'],'Inputs','++');
add('built-in/TransferFcn',[pl '/G'],150,60,80,40);
set_param([pl '/G'],'Numerator',mat2str(numG),'Denominator',mat2str(denG));
add('built-in/Outport',[pl '/dalpha'],270,70,20,14);
add_line(pl,'dae/1','Soma/1'); add_line(pl,'dpert/1','Soma/2','autorouting','on');
add_line(pl,'Soma/1','G/1'); add_line(pl,'G/1','dalpha/1');

%% ---------- TOPO ----------
add('simulink/Sources/Step',[mdl '/Ref'],30,100,40,30);
set_param([mdl '/Ref'],'Time','0','Before','0','After','0');   % REGULATORIO r=0
set_param(pid,'Position',[110 90 210 160]);
add('simulink/Discontinuities/Saturation',[mdl '/SatV'],250,105,40,30);
set_param([mdl '/SatV'],'UpperLimit',num2str(Umax),'LowerLimit',num2str(-Umax));
% comp. inversa da deadzone: v + A*sign(v) + B
add('simulink/Math Operations/Sign',[mdl '/Sgn'],320,165,26,26);
add('simulink/Math Operations/Gain',[mdl '/GA'],370,165,40,26); set_param([mdl '/GA'],'Gain',num2str(A));
add('simulink/Sources/Constant',[mdl '/Bconst'],370,215,40,26); set_param([mdl '/Bconst'],'Value',num2str(B));
add('simulink/Math Operations/Sum',[mdl '/Comp'],430,110,24,40); set_param([mdl '/Comp'],'Inputs','+++');
add('simulink/Discontinuities/Dead Zone',[mdl '/DZ motor'],490,108,50,30);
set_param([mdl '/DZ motor'],'LowerValue',num2str(-DZN),'UpperValue',num2str(DZP));
add('simulink/Sources/Step',[mdl '/Perturbacao'],490,185,50,30);
set_param([mdl '/Perturbacao'],'Time',num2str(t_dist),'Before','0','After',num2str(d_pwm));
set_param(pl,'Position',[580 100 680 165]);
add('simulink/Math Operations/Gain',[mdl '/Rad2Deg'],720,110,50,28); set_param([mdl '/Rad2Deg'],'Gain','180/pi');
add('simulink/Sinks/To Workspace',[mdl '/AlphaOut'],800,105,70,34);
set_param([mdl '/AlphaOut'],'VariableName','alpha_deg','SaveFormat','Timeseries');
add('simulink/Sinks/Scope',[mdl '/Escopo'],800,175,40,34);
add('simulink/Sinks/To Workspace',[mdl '/Uout'],430,40,60,30);
set_param([mdl '/Uout'],'VariableName','u','SaveFormat','Timeseries');

% ligacoes do topo
add_line(mdl,'Ref/1','PID 2DOF Discreto - FT/1','autorouting','on');          % R
add_line(mdl,'PID 2DOF Discreto - FT/1','SatV/1','autorouting','on');          % U
add_line(mdl,'SatV/1','Comp/1','autorouting','on');
add_line(mdl,'SatV/1','Sgn/1','autorouting','on');
add_line(mdl,'SatV/1','Uout/1','autorouting','on');
add_line(mdl,'Sgn/1','GA/1'); add_line(mdl,'GA/1','Comp/2','autorouting','on');
add_line(mdl,'Bconst/1','Comp/3','autorouting','on');
add_line(mdl,'Comp/1','DZ motor/1','autorouting','on');
add_line(mdl,'DZ motor/1','Planta Furuta/1','autorouting','on');              % dae
add_line(mdl,'Perturbacao/1','Planta Furuta/2','autorouting','on');           % dpert
add_line(mdl,'Planta Furuta/1','Rad2Deg/1','autorouting','on');
add_line(mdl,'Rad2Deg/1','AlphaOut/1','autorouting','on');
add_line(mdl,'Rad2Deg/1','Escopo/1','autorouting','on');
add_line(mdl,'Planta Furuta/1','PID 2DOF Discreto - FT/2','autorouting','on'); % realimenta Y (rad)

set_param(mdl,'Solver','ode45','StopTime','8','SaveOutput','on','SaveTime','on');
try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl);
fprintf('Modelo "%s.slx" criado (padrao de subsistemas). Rode: out=sim(''%s'');\n',mdl,mdl);
end
