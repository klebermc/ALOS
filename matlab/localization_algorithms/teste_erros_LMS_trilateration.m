% Esse algoritmo foi usado para, dado um ponto virtual a área de testes, tentar decidir qual a
% configuração de módulos emissores no ambiente que gera o menor erro final.

% Algoritmo para fazer a decisão:
 

close all;
clear;
clc;

error=[];

%posição dos módulos testes
% stationary_modules_positions = [0 0 0; 0 3 0; 3 3 0; 3 0 0.2];
% stationary_modules_positions = [0 0 0; 0 2 0; 2 2 0; 2 0 0; 2 1 1]; %melhor embaixo
% stationary_modules_positions = [0 0 3; 1.5 0 2; 0 1.5 2; 0 3 3; 3 3 3; 3 0 3; 3 1.5 2; 1.5 3 2]; %melhor em cima
% stationary_modules_positions = [0 0 3; 0 3 3; 3 3 3; 3 0 3; 1.5 3 2];
% stationary_modules_positions = [0 0 0; 0.1 1 0.1; 0 2 0; 2 0 0; 1.9 1 0.1; 2 2 0]; %segundo melhor
% 
% stationary_modules_positions = [
%     0    0.05 2.5;
%     2.35 0.05 2.5;
%     2.35 3.05 2.5;
%     0    2.88 2.5;
%     2.90 1.60 2;
%     1.17 3.05 3];
stationary_modules_positions = [
                                0.000  0.05  2.48;
                                1.285  0.05  2.96;
                                2.535  0.05  2.48; 
                                2.915  1.60  1.99; 
                                2.570  3.05  2.48; 
                                1.260  3.05  2.96;
                                -0.05  2.90  2.48;
                                -0.47  1.52  1.77
                                ];
num_statinary_modules = size(stationary_modules_positions,1);

%ponto escolhido para teste
% x=1;y=1;z=2;
x=1.25 ;%+ 0.3*randn;
y=1.5-0.55 ;%+ 0.3*randn;
z=0.5 ;%+ 0.3*randn;

measurements_m=[];
measurements_new_m=[];

figure
subplot(3,1,1);
grid on;
plot([1 100],[x x]);
axis([1 100 0 3])
subplot(3,1,2);
grid on;
plot([1 100],[y y]);
axis([1 100 0 3])
subplot(3,1,3);
grid on;
plot([1 100],[z z]);
axis([1 100 0 3])

% %inicia o plot da figura
% figure;
% plot3(stationary_modules_positions(1,1),stationary_modules_positions(1,2),stationary_modules_positions(1,3),'rx');
% hold on;
% plot3([stationary_modules_positions(1,1) stationary_modules_positions(1,1)],[stationary_modules_positions(1,2) stationary_modules_positions(1,2)],[0 stationary_modules_positions(1,3)],'k','LineWidth',1.5);
% for i=2:num_statinary_modules
%     plot3(stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3),'rx');
%     plot3([stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'k','LineWidth',1.5);
% end
% grid on;
% plot3(x,y,z,'m*')
% axis([0 3 0 3 0 3]);

%Calcula as distâncias entre o ponto escolhido e posição dos n módulos
for i=1:num_statinary_modules
    m(i) = sqrt( (x-stationary_modules_positions(i,1))^2 + (y-stationary_modules_positions(i,2))^2 + (z-stationary_modules_positions(i,3))^2 );
end
    
for i=1:100
    
%     dp = 1.0131; %desvio padrao em centímetros
    dp=5;
    distance_readings=[];
    aleat=round(1 + (8-1).*rand(1,1));
    for j=1:num_statinary_modules
        new_m(j) = m(j) ;% + (dp/100)*randn;  %adicionando ruido às leituras de medidas
%         if j==aleat
%             new_m(j)=new_m(j)+0.1;
%         end
        distance_readings = [distance_readings,new_m(j)];
    end
    
    measurements_m = [measurements_m;m];
    measurements_new_m=[measurements_new_m;new_m];
    
%     stationary_modules_positions
%     distance_readings
    
%     [module_x,module_y,module_z] = trilateration(stationary_modules_positions, distance_readings);
%     [module_x,module_y,module_z,erro] = taylor_series_test( stationary_modules_positions , distance_readings, [0,0,0] );
%     [erro, aleat]
%     [mts_x,mts_y,mts_z] = round_TS(stationary_modules_positions , distance_readings);
    
%     % ----------------------------------------------------------------------------------------------
%     %comparing various start points to taylor series algorithm
%     %the difference is in micrometers (um), there is no pratical difference, only numerical difference
%     [mt1_x,mt1_y,mt1_z] = taylor_series( stationary_modules_positions , distance_readings, [module_x,module_y,module_z] );
%     [mt2_x,mt2_y,mt2_z] = taylor_series( stationary_modules_positions , distance_readings, [10,10,0] );
    [mt3_x,mt3_y,mt3_z] = taylor_series( stationary_modules_positions , distance_readings, [1.25,1.50,0] );
% 
%     sprintf('%.6f ', abs([mt1_x,mt1_y,mt1_z] -  [mt2_x,mt2_y,mt2_z]))
%     sprintf('%.6f ', abs([mt1_x,mt1_y,mt1_z] -  [mt3_x,mt3_y,mt3_z]))
%     
%     subplot(3,1,1);
%         hold on;
%         plot(i,module_x,'ko');
%         plot(i,mt1_x,'bp');
%         plot(i,mt2_x,'rh');
%         plot(i,mt3_x,'m*');
%         hold off;
%     subplot(3,1,2);
%         hold on;
%         plot(i,module_y,'ko');
%         plot(i,mt1_y,'bp');
%         plot(i,mt2_y,'rh');
%         plot(i,mt3_y,'m*');
%         hold off;
%     subplot(3,1,3);
%         hold on;
%         plot(i,module_z,'ko');
%         plot(i,mt1_z,'bp');
%         plot(i,mt2_z,'rh');
%         plot(i,mt3_z,'m*');
%         hold off;
%     % ----------------------------------------------------------------------------------------------

    % ----------------------------------------------------------------------------------------------
%     %comparing various standard deviation values inside taylor_series algorithm
%     %does not make any difference
%     [mt1_x,mt1_y,mt1_z] = taylor_series_Qmat( stationary_modules_positions , distance_readings, [1.0,1.0,0] , 0.10);
%     [mt2_x,mt2_y,mt2_z] = taylor_series_Qmat( stationary_modules_positions , distance_readings, [1.0,1.0,0] , 0.25);
%     [mt3_x,mt3_y,mt3_z] = taylor_series_Qmat( stationary_modules_positions , distance_readings, [1.0,1.0,0] , 0.50);

%     error = [error, sqrt( (x-mt1_x)^2 + (y-mt1_y)^2 + (z-mt1_z)^2 )];
%     sprintf('%.3f  \t  %.3f  \t  %.3f\n', sqrt( (x-mt1_x)^2 + (y-mt1_y)^2 + (z-mt1_z)^2 ), sqrt( (x-mt2_x)^2 + (y-mt2_y)^2 + (z-mt2_z)^2 ), sqrt( (x-mt3_x)^2 + (y-mt3_y)^2 + (z-mt3_z)^2 ))
%     sprintf('%.6f ', abs([mt1_x,mt1_y,mt1_z] -  [mt2_x,mt2_y,mt2_z]))
%     sprintf('%.6f ', abs([mt1_x,mt1_y,mt1_z] -  [mt3_x,mt3_y,mt3_z]))
%     
%     subplot(3,1,1);
%         hold on;
%         plot(i,module_x,'ko');
%         plot(i,mt1_x,'bp');
%         plot(i,mt2_x,'rh');
%         plot(i,mt3_x,'m*');
%         hold off;
%     subplot(3,1,2);
%         hold on;
%         plot(i,module_y,'ko');
%         plot(i,mt1_y,'bp');
%         plot(i,mt2_y,'rh');
%         plot(i,mt3_y,'m*');
%         hold off;
%     subplot(3,1,3);
%         hold on;
%         plot(i,module_z,'ko');
%         plot(i,mt1_z,'bp');
%         plot(i,mt2_z,'rh');
%         plot(i,mt3_z,'m*');
%         hold off;
    % ----------------------------------------------------------------------------------------------

%     plot3(module_x,module_y,module_z,'bo');
%     plot3([x module_x],[y module_y], [z module_z],'k');
    
%     disp('--------------------')
%     disp(distance_readings)
%     disp([x,y,z])
%     disp([module_x,module_y,module_z])
%     disp(sqrt( (x-module_x)^2 + (y-module_y)^2 + (z-module_z)^2 ))
%     disp('--------------------')
%     pause
%     error = [error, sqrt( (x-module_x)^2 + (y-module_y)^2 + (z-module_z)^2 )];
end
% hold off;
% 
% figure;
% subplot(3,1,1);
% plot(error);
% grid on;
% title('Position Measurement Error');
% ylabel('error (meters)');
% legend(strcat('max error = ', num2str(max(error))));
% disp(strcat('max error = ', num2str(max(error))));
% 
% subplot(3,1,2);
% hold on;
% % plot(error);
% for i=1:num_statinary_modules
%     plot(measurements_m(:,i),'DisplayName',strcat('ideal sensor=',num2str(i)));
%     plot(measurements_new_m(:,i),'DisplayName',strcat('real  sensor=',num2str(i)));
% end
% grid on;
% legend('show');
% hold off;
% 
% subplot(3,1,3);
% hist(error);
% legend(strcat('media=',num2str(mean(error))));
% disp(strcat('media=',num2str(mean(error))));