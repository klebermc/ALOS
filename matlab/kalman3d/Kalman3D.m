% *************** Kalman Filter simulation *****************
% Kléber Cabral
% 05/08/2017
%
% This simulation uses the error model to predict the accel x,y and gyro z biases
%
% The states are:  X = [Vx, Vy, Px, Py, psi]'
%
% Reference: 

close all
clear all;

%Simulation parameters and variables
dt=0.1;

time=0:0.1:120;
y=sin(time/2);
% plot(time,y);
% title('accelx');


%**************************************************************
%All the states that are somehow needed by Kalman Filter
bias_ax=0.1;
bias_ay=-0.1;
bias_wz=0.01;

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
beta = 0.001;         %parameter to avoid filter divergence
R = [(10*noise_px_LS)^2  0 0; 0 (10*noise_py_LS)^2  0; 0 0 (10*noise_psi_comp)^2 ]; % covariance matrix of Location System and Compass N_LS
Q = [(100*noise_ax_INS)^2 0 0; 0 (100*noise_ay_INS)^2 0; 0 0 (100*noise_wz_INS)^2 ]; %covariance matrix of the IMU noise vector N_INS
H = [zeros(3,2) eye(3) zeros(3,3)];


HISTORY_Bias=[];
HISTORY_P=diag(P0);
%**************************************************************


%******** Estados verdadeiros **********
%as inputs (vetor U)
axb = y;
ayb = 0*time;
wzb = 0*time;
clear y

for k=1:length(axb)/4
    axb(k)=0;
end

for k=(fix((length(axb)*3)/4)):length(axb)
    axb(k)=0;
end
% inicialização dos estados (vetor X)
Xv=[0;0;0;0;0];

%Simulação da dinamica do sistema verdadeiro
for k=2:length(time)
    U=[axb(k);ayb(k);wzb(k)]; %Input
    Xv(:,k) = euler_integration(Xv(:,k-1), U, dt);
end
%**********************************

%******** Estados do INS **********
%Inputs
for i=1:length(axb)
    %a_INS = a + bias + noise
    axb_INS(i) = axb(i) + randn*(bias_ax+(i/100)) + (noise_ax_INS)*randn; 
%     axb_INS(i) = axb(i) + bias_ax + noise_ax_INS*randn; 
    ayb_INS(i) = ayb(i) + bias_ay + (noise_ay_INS)*randn; 
    wzb_INS(i) = wzb(i) + bias_wz + (noise_wz_INS)*randn; 
end

% inicialização dos estados (vetor X)
X_INS=[0;0;0;0;0];
%**********************************


%******** Medida Auxiliar *********
Px_LS=[];
Py_LS=[];
Psi_COMP=[];
for i=10:10:max(size(Xv))
    Px_LS       = [Px_LS   , Xv(3,i-5) + noise_px_LS*randn    ]; %medida X ruidosa
    Py_LS       = [Py_LS   , Xv(4,i-5) + noise_py_LS*randn    ];
    Psi_COMP    = [Psi_COMP, Xv(5,i) + noise_psi_comp*randn ]; %radians
end
%**********************************

%******* O Modelo de Erro ********
%     %X = A_E * X_E + B_E * N_INS
%     %Y = H * X_E + N_LS
% 
%     CS = [cos(X_INS(5)) -sin(X_INS(5)); sin(X_INS(5)) cos(X_INS(5))];
% 
%     A_E = [ 
%             0 0 0 0 0 C(1,:) 0;
%             0 0 0 0 0 C(2,:) 0;
%             1 0 0 0 0 0   0  0;
%             0 1 0 0 0 0   0  0;
%             0 0 0 0 0 0   0  1;
%             0 0 0 0 0 0   0  0;
%             0 0 0 0 0 0   0  0;
%             0 0 0 0 0 0   0  0
%             ];
% 
%     B_E = [ 
%            C(1,:) 0;
%            C(2,:) 0;
%            0    0 0;
%            0    0 0;
%            0    0 1;
%            0    0 0;
%            0    0 0;
%            0    0 0
%            ];
% 
%     %Discretizando
%     Ad_E = expm(A_E*dt);
%     Bd_E = expm(A_E*dt) * B_E;
%*********************************


%Agora é o loop que meu programa vai executar de fato
for k=1:length(axb_INS)-1  
    % ****** Atualzacao do filtro de Kalman ******
    if mod(k,10) == 0
       [ X_INS(:,k), P, Bias ] = kalman_filter( X_INS(:,k), Px_LS(k/10) , Py_LS(k/10), Psi_COMP(k/10), Bias, H, Q, R, beta, P, P0, 'update' );
    end
    HISTORY_P = [HISTORY_P , diag(P)];
    % ------------------------------------------
    
    % ****** Propagação do filtro de Kalman ******
    U_INS = [axb_INS(k);ayb_INS(k);wzb_INS(k)] - Bias; %inputs with Bias compensation
    X_INS(:,k+1) = euler_integration(X_INS(:,k), U_INS, dt);
    
    % ****** Ultima etapa da propagação P_minus_k+1 ******
    [ X_INS(:,k), P, Bias ] = kalman_filter( X_INS(:,k), 0, 0, 0, Bias, H, Q, R, beta, P, P0, 'propagation' );
    HISTORY_Bias = [HISTORY_Bias, Bias];
    % ------------------------------------------    
    
end

figure(1); 
subplot(3,1,1); hold on; grid on; plot(time, axb); plot(time, axb_INS); legend('ax verdadeiro','ax INS');
subplot(3,1,2); hold on; grid on; plot(time, ayb); plot(time, ayb_INS); legend('ay verdadeiro','ay INS');
subplot(3,1,3); hold on; grid on; plot(time, wzb); plot(time, wzb_INS); legend('wz verdadeiro','wz INS');

figure(2);
subplot(3,1,1); 
    hold on; grid on; 
    plot(time,(X_INS(1,:))');plot(time,(X_INS(2,:))');plot(time,(Xv(1,:))');plot(time,(Xv(2,:))');
    legend('Vx INS - estimado','Vy INS - estimado','Vx - verdadeiro','Vy - verdadeiro','Location','Best');
subplot(3,1,2); 
    hold on; grid on; 
    plot(time,(X_INS(3,:))');plot(time,(X_INS(4,:))');plot(time,(Xv(3,:))');plot(time,(Xv(4,:))');
    legend('Px INS - estimado','Py INS - estimado','Px - verdadeiro','Py - verdadeiro','Location','Best');
    for i=1:length(Px_LS)
        plot(i, Px_LS(i), 'kx');
        plot(i, Py_LS(i), 'kx');
    end
subplot(3,1,3); 
    hold on; grid on; 
    plot(time,(X_INS(5,:))');plot(time,(Xv(5,:))');
    legend( 'Psi INS - estimado','Psi - verdadeiro','Location','Best');
    

figure(3);
plot(HISTORY_Bias');
grid on;
legend('Bias ax','Bias ay','Bias wz');

figure(4);
subplot(4,2,1); plot(time,HISTORY_P(1,:)); grid on; title('Vx');
subplot(4,2,2); plot(time,HISTORY_P(2,:)); grid on; title('Vy');
subplot(4,2,3); plot(time,HISTORY_P(3,:)); grid on; title('Px');
subplot(4,2,4); plot(time,HISTORY_P(4,:)); grid on; title('Py');
subplot(4,2,5); plot(time,HISTORY_P(5,:)); grid on; title('Psi');
subplot(4,2,6); plot(time,HISTORY_P(6,:)); grid on; title('biasax');
subplot(4,2,7); plot(time,HISTORY_P(7,:)); grid on; title('biasay');
subplot(4,2,8); plot(time,HISTORY_P(8,:)); grid on; title('biaswz');

for k=1:max(size(HISTORY_P)); DV(:,k)=sqrt(HISTORY_P(:,k)); end;
figure(5);
subplot(3,1,1);
plot(time,X_INS(3,:)-Xv(3,:),'r-',time,3*DV(3,:),'b-',time,-DV(3,:)*3,'b-');
legend('Erro na posicao em x do INS [m]','+-3 * "Desvio Padrao" em x [m]','Location','Best');
xlabel('Tempo (s)');ylabel('posição em x (m)'); grid on;
title('Erro e desvio padrao na estimativa de posicao em x');

subplot(3,1,2);
plot(time,X_INS(4,:)-Xv(4,:),'r-',time,3*DV(4,:),'b-',time,-DV(4,:)*3,'b-');
legend('Erro na posicao em y do INS [m]','+-3 * "Desvio Padrao" em y [m]','Location','Best');
xlabel('Tempo (s)');ylabel('posicao em y (m)'); grid on;
title('Erro e desvio padrao na estimativa de posicao em y');

subplot(3,1,3);
plot(time,(X_INS(5,:)-Xv(5,:))*180/pi,'r-',time,3*DV(5,:)*180/pi,'b-',time,-DV(5,:)*3*180/pi,'b-');
legend('Erro no angulo de guinada [graus]','+-3 * "Desvio Padrao" em psi [graus]','Location','Best');
xlabel('Tempo (s)'); ylabel('angulo de Guinada (graus)'); grid on;
title('Erro e desvio padrao na estimativa do angulo de guinada');

figure(6);
hold on; grid on;
plot(time,X_INS(3,:)-Xv(3,:),'r');
plot(time,X_INS(4,:)-Xv(4,:),'b');
title('Erro na estimativa de posicao em x e y');
legend('Erro pos x [m]','Erro pos y [m]','Location','Best');
