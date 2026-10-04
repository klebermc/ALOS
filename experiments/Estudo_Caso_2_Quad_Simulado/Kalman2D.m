function [sys,x0,str,ts] = Kalman2D(t,x,u,flag)
% *************** Kalman Filter S-Function*****************
% Kléber Cabral
% 05/08/2017
%
% This simulation uses the error model to predict the accel x,y and gyro z biases
%
% The states are:  X = [Vx, Vy, Px, Py, psi]'

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


%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = []; % No continuous states
str = []; % No state ordering
ts = [-1 0]; % Inherited sample time - sample time: [period, offset]

%Simulation parameters and variables
dt=0.1;

% Access global variables from workspace
global P0;
global P;
global beta;
global R;
global Q;
global H;
global Bias;
global X_INS;
global HISTORY_Bias;
global HISTORY_P;

global rising_det_sig_10Hz;
global rising_det_sig_1Hz;

global k;
global time;

k=1;
time=[];
rising_det_sig_10Hz=0;
rising_det_sig_1Hz=0;

%**************************************************************
%All the states that are somehow needed by Kalman Filter

%Vector N_INS
noise_ax_INS = 0.0475; %The standard deviation
noise_ay_INS = 0.0475;
noise_wz_INS = 0.007;

%Vector N_LS
noise_px_LS = 0.25;   %The standard deviation
noise_py_LS = 0.25;
noise_psi_comp = deg2rad(1);

Bias = [0.1;0;0]; %initialization of the bias estimative needed by KF

P0 = eye(8);        %initial state error covariance matrix
P0(1,1)=50;
P0(2,2)=50;
P0(3,3)=50;
P0(4,4)=50;
P0(5,5)=deg2rad(10);
P0(6,6)=10;
P0(7,7)=10;
P0(8,8)=10;

P  = P0; 
beta = 0.01;         %parameter to avoid filter divergence
%R = [(10*noise_px_LS)^2  0 0; 0 (10*noise_py_LS)^2  0; 0 0 (10*noise_psi_comp)^2 ]; % covariance matrix of Location System and Compass N_LS
%Q = [(100*noise_ax_INS)^2 0 0; 0 (100*noise_ay_INS)^2 0; 0 0 (100*noise_wz_INS)^2 ]; %covariance matrix of the IMU noise vector N_INS
R = [(noise_px_LS)^2  0 0; 0 (noise_py_LS)^2  0; 0 0 (noise_psi_comp)^2 ]; % covariance matrix of Location System and Compass N_LS
Q = [(noise_ax_INS)^2 0 0; 0 (noise_ay_INS)^2 0; 0 0 (noise_wz_INS)^2 ]; %covariance matrix of the IMU noise vector N_INS
H = [zeros(3,2) eye(3) zeros(3,3)];

HISTORY_Bias=[];
HISTORY_P=diag(P0);
%**************************************************************

%******** Estados do INS **********
% inicialização dos estados (vetor X)
X_INS=[0;0;0;0;0];
%**********************************
% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u)
global clientID;
global vrep;

% vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)

% Access global variables from workspace
global P0;
global P;
global beta;
global R;
global Q;
global H;
global Bias;
global X_INS;
global HISTORY_Bias;
global HISTORY_P;

global rising_det_sig_10Hz;
global rising_det_sig_1Hz;

global k;
global time;

dt=0.1;

%IMU
axb_INS(k) = u(1);
ayb_INS(k) = u(2);
wzb_INS(k) = u(3);

%Location System and Compass
Px_LS(k) = u(4);
Py_LS(k) = u(5);
Psi_COMP(k) = u(6);

% ****** Atualzacao do filtro de Kalman ******
if rising_det_sig_1Hz < u(8) %the signal went from 0 to 1
   [ X_INS(:,k), P, Bias ] = kalman_filter( X_INS(:,k), Px_LS(k) , Py_LS(k), Psi_COMP(k), Bias, H, Q, R, beta, P, P0, 'update' );
end
% ------------------------------------------

% ****** Propagação do filtro de Kalman ******
if rising_det_sig_10Hz < u(7) %the signal went from 0 to 1
    time=[time, t];
    
    U_INS = [axb_INS(k);ayb_INS(k);wzb_INS(k)] - Bias; %inputs with Bias compensation
    X_INS(:,k+1) = euler_integration(X_INS(:,k), U_INS, dt);

    % ****** Ultima etapa da propagação P_minus_k+1 ******
    [ X_INS(:,k), P, Bias ] = kalman_filter( X_INS(:,k), 0, 0, 0, Bias, H, Q, R, beta, P, P0, 'propagation' );
    HISTORY_Bias = [HISTORY_Bias, Bias];
    HISTORY_P = [HISTORY_P , diag(P)];
    k=k+1;
end

rising_det_sig_10Hz=u(7);
rising_det_sig_1Hz=u(8);
% ------------------------------------------    
% vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
sys = [X_INS(3,k), X_INS(4,k), X_INS(5,k)];
% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

% Access global variables from workspace
global P0;
global P;
global beta;
global R;
global Q;
global H;
global Bias;
global X_INS;
global HISTORY_Bias;
global HISTORY_P;

global k;
global time;

save('kalman_variables.mat','P0','P','beta','R', 'Q', 'H', 'Bias', 'X_INS', 'HISTORY_Bias', 'HISTORY_P', 'time');

% figure(1);
% subplot(3,1,1); 
%     hold on; grid on; 
%     plot(time,(X_INS(1,:))');plot(time,(X_INS(2,:))');plot(time,(Xv(1,:))');plot(time,(Xv(2,:))');
%     legend('Vx INS - estimado','Vy INS - estimado','Vx - verdadeiro','Vy - verdadeiro','Location','Best');
% subplot(3,1,2); 
%     hold on; grid on; 
%     plot(time,(X_INS(3,:))');plot(time,(X_INS(4,:))');plot(time,(Xv(3,:))');plot(time,(Xv(4,:))');
%     legend('Px INS - estimado','Py INS - estimado','Px - verdadeiro','Py - verdadeiro','Location','Best');
%     for i=1:length(Px_LS)
%         plot(i, Px_LS(i), 'kx');
%         plot(i, Py_LS(i), 'kx');
%     end
% subplot(3,1,3); 
%     hold on; grid on; 
%     plot(time,(X_INS(5,:))');plot(time,(Xv(5,:))');
%     legend( 'Psi INS - estimado','Psi - verdadeiro','Location','Best');

% figure(2);
% plot(HISTORY_Bias');
% grid on;
% legend('Bias ax','Bias ay','Bias wz');
% 
% figure(3);
% subplot(4,2,1); plot(time,HISTORY_P(1,:)); grid on; title('Vx');
% subplot(4,2,2); plot(time,HISTORY_P(2,:)); grid on; title('Vy');
% subplot(4,2,3); plot(time,HISTORY_P(3,:)); grid on; title('Px');
% subplot(4,2,4); plot(time,HISTORY_P(4,:)); grid on; title('Py');
% subplot(4,2,5); plot(time,HISTORY_P(5,:)); grid on; title('Psi');
% subplot(4,2,6); plot(time,HISTORY_P(6,:)); grid on; title('biasax');
% subplot(4,2,7); plot(time,HISTORY_P(7,:)); grid on; title('biasay');
% subplot(4,2,8); plot(time,HISTORY_P(8,:)); grid on; title('biaswz');
% 
% for k=1:max(size(HISTORY_P)); DV(:,k)=sqrt(HISTORY_P(:,k)); end;
% figure(4);
% subplot(3,1,1);
% plot(time,X_INS(3,:)-Xv(3,:),'r-',time,3*DV(3,:),'b-',time,-DV(3,:)*3,'b-');
% legend('Erro na posicao em x do INS [m]','+-3 * "Desvio Padrao" em x [m]','Location','Best');
% xlabel('Tempo (s)');ylabel('posição em x (m)'); grid on;
% title('Erro e desvio padrao na estimativa de posicao em x');
% 
% subplot(3,1,2);
% plot(time,X_INS(4,:)-Xv(4,:),'r-',time,3*DV(4,:),'b-',time,-DV(4,:)*3,'b-');
% legend('Erro na posicao em y do INS [m]','+-3 * "Desvio Padrao" em y [m]','Location','Best');
% xlabel('Tempo (s)');ylabel('posicao em y (m)'); grid on;
% title('Erro e desvio padrao na estimativa de posicao em y');
% 
% subplot(3,1,3);
% plot(time,(X_INS(5,:)-Xv(5,:))*180/pi,'r-',time,3*DV(5,:)*180/pi,'b-',time,-DV(5,:)*3*180/pi,'b-');
% legend('Erro no angulo de guinada [graus]','+-3 * "Desvio Padrao" em psi [graus]','Location','Best');
% xlabel('Tempo (s)'); ylabel('angulo de Guinada (graus)'); grid on;
% title('Erro e desvio padrao na estimativa do angulo de guinada');
% 
% figure(6);
% hold on; grid on;
% plot(time,X_INS(3,:)-Xv(3,:),'r');
% plot(time,X_INS(4,:)-Xv(4,:),'b');
% title('Erro na estimativa de posicao em x e y');
% legend('Erro pos x [m]','Erro pos y [m]','Location','Best');
sys = [];
% end mdlTerminate