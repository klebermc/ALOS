function [sys,x0,str,ts] = sensores(t,x,u,flag)
% Dispatch the flag. The switch function controls the calls to
% S-function routines at each simulation stage.
switch flag,
	case 0
		[sys,x0,str,ts] = mdlInitializeSizes; % Initialization
		
	case 3
		sys = mdlOutputs(t,x,u); % Calculate outputs
	
	case 9
		sys = mdlTerminate(t,x,u);
	
	case { 1, 2, 4 }
		sys = []; % Unused flags
	
	otherwise
	error(['Unhandled flag = ',num2str(flag)]); % Error handling
end;
% End of function vrep_comm.


%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()
global Bias Qimu Rgps Rbus ic U
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = []; % No continuous states
str = []; % No state ordering
ts = [-1 0]; % Inherited sample time - sample time: [period, offset]

% speicfy that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%

function sys = mdlOutputs(t,x,u)
global Bias Qimu Rgps Rbus ic U

xGPS  = u(1);    % Posicao no eixo x verdadeira [m]
yGPS  = u(2);    % Posicao no eixo y verdadeira [m]
phi_BUS = u(3);    % Angulo de guinada verdadeiro [rad]
Axbm   = u(4);    % Aceleracao no eixo x do corpo no sistema verdadeiro [m/s^2]
Aybm   = u(5);    % Aceleracao no eixo y do corpo no sistema verdadeiro [m/s^2]
Wzm    = u(6);    % Velocidade angular no eixo z do corpo no sistema verdadeiro [rad/s]
Ym(1,:) = xGPS;
Ym(2,:) = yGPS;
Ym(3,:) = phi_BUS;

DT = 0.25;
DTgps=4*DT; 
Ngps=fix(DTgps/DT); 

Ne=8;              % Número de estados estimados pelo FK
Nre=3;             % Número de entradas ruidosas na equação de estados
Nm=3;              % Número de medidas usadas no FK
Xe0=zeros([Ne 1]); % estimativa do estado inicial do FK
%P0=diag([100 100 100 100 (90*pi/180)^2 25 25 (25*pi/180)^2]); % incerteza na estimativa do estado inicial do FK
P0=diag([10 10 12 12 0.01 10 10 5]); % incerteza na estimativa do estado inicial do FK
%P0=diag([0 0 0 0 0 0 0 0]);
%P0 = diag([5, 5, 6, 6, 0.05, 5, 5, 2.5]);
% Loop de simulação do Filtro de Kalman

Xins=zeros([5 1]);
%Xins(:,1)=X0;
Bias_est=Xe0(6:8,1);

Ae=zeros([Ne Ne]); Ae(3,1)=1; Ae(4,2)=1; Ae(5,8)=1;
Be=zeros([Ne Nre]);
Be(5,3)=1;
Ce=[zeros([3 2]), eye([3 6])];
De=zeros([Nm Nre]);
R=[[Rgps, [0; 0]]; 0 0 Rbus];
Beta=P0/200;

%Beta(8,8)=0.01;
%Beta = 0.005;
%for k=1:Np;
    %fprintf(' %d',k); %if k==20*fix(k/20); fprintf('\n'); end;
    % Simulação do INS
    U=[Axbm Aybm Wzm]' - Bias_est;
    [tt,xt]=ode45('corpo3d_f',[0:DT/10:DT],Xins);
    Xins=xt';
    Bias_esti=Bias_est;
    % Etapa de Propagação do FK
    % Fazer P(k+1)=P(k)+Bd*(Q*DT)*(Bd)'
    Fi=Xins(5);
    Be(1:2,1:2)=[cos(Fi), -sin(Fi); sin(Fi), cos(Fi)];
    Ae(1:2,6:7)=[cos(Fi), -sin(Fi); sin(Fi), cos(Fi)];
    sys_se=ss(Ae,Be,Ce,De);
    sys_se_d=c2d(sys_se,DT);
    Aed=sys_se_d.a;
    Bed=sys_se_d.b;
    Pk1=Aed*P0*(Aed)'+Bed*(Qimu*DT)*(Bed)'+Beta;
    Pk1(8,1)=1;
    % Etapa de Atualização feita quando Ygps é disponível
    ic=ic+1; 
    if ic==Ngps;
       Gfk=Pk1*Ce'*inv(Ce*Pk1*Ce'+R); % R deve ter dimensão [Nm,Nm]
       Pk1=(eye(size(Pk1))-Gfk*Ce)*Pk1; % P(k+)
       dY=Xins(3:5)-Ym;
       Xek=[zeros([5 1]); Bias_esti]+Gfk*dY; % Xe(k+)
       Xins=Xins-Xek(1:5); % Corrige Xins
       Bias_esti=Xek(6:8,1); % Corrige estimativa de Bias
       ic=0;
       %fprintf(' --> Passo GPS = %d\n',kgps);
    end;
    P0=Pk1;
    
%end;
sys = [Xins(1,1),Xins(2,1),Xins(3,1),Xins(4,1),Xins(5,1)];
% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
sys = [];
% end mdlTerminate