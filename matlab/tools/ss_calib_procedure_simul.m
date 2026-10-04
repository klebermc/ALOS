close all;
clear;

error=[];

%posição dos módulos testes
stationary_modules_positions = [
                                0.000  0.05  2.48;
                                1.285  0.05  2.96;
                                2.535  0.05  2.48; 
                                2.915  1.60  1.99; 
                                2.570  3.05  2.48; 
                                1.260  3.05  2.96;
                                -0.05  2.90  2.48%;
                                -0.47  1.52  1.77
                                ];

num_statinary_modules = size(stationary_modules_positions,1);

%ponto escolhido para teste
x=1.25;
y=1.5 ;
z=0.5 ;

measurements_m=[];
measurements_new_m=[];

%inicia o plot da figura
% figure;
% plot3(stationary_modules_positions(1,1),stationary_modules_positions(1,2),stationary_modules_positions(1,3),'rx');
% hold on;
% plot3([stationary_modules_positions(1,1) stationary_modules_positions(1,1)],[stationary_modules_positions(1,2) stationary_modules_positions(1,2)],[0 stationary_modules_positions(1,3)],'k','LineWidth',1.5);
% for i=2:num_statinary_modules
%     plot3(stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3),'rx');
%     plot3([stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'k','LineWidth',1.5);
% end
% axis([min(stationary_modules_positions(:,1)) max(stationary_modules_positions(:,1)) min(stationary_modules_positions(:,2)) max(stationary_modules_positions(:,2)) 0 max(stationary_modules_positions(:,3))]); %The axis limits
% xlabel('X[metros]');
% ylabel('Y[metros]');
% zlabel('Z[metros]');
% grid on;
% plot3(x,y,z,'m*')

%Calcula as distâncias entre o ponto escolhido e posição dos n módulos
for i=1:num_statinary_modules
    m(i) = sqrt( (x-stationary_modules_positions(i,1))^2 + (y-stationary_modules_positions(i,2))^2 + (z-stationary_modules_positions(i,3))^2 );
    time_measurements(i) = m(i) / 340;
end
    
%The position of the module for calibration
x_calib = 1.25;
y_calib = 1.50;
z_calib = 0.50; %ground robot

%Iterate, testing different values for the the sound speed until I found the one with smaller error.
soundspeed = 200;
best_values=[100 soundspeed];
distance_measurements=[];
%error=[];
eixo_x=[];
eixo_y=[];
while soundspeed<400

    for i=1:size(time_measurements,2)
        distance_measurements(i) = time_measurements(i) * soundspeed;
    end

    [module_x,module_y,module_z] = LMS(stationary_modules_positions, distance_measurements);
%    [module_x,module_y,module_z] = taylor_series(stationary_modules_positions, distance_measurements, [module_x,module_y,module_z] );
    dist_from_real_point = sqrt( (x_calib-module_x)^2 + (y_calib-module_y)^2 + (z_calib-module_z)^2 );
    if dist_from_real_point < best_values(1)
        best_values(1)=dist_from_real_point;
        best_values(2)=soundspeed;
    end

    %error=[error; sqrt( (x_calib-module_x)^2 + (y_calib-module_y)^2 ), dist_from_real_point];
    if mod(int32(soundspeed*100),500)==0
        %Show the value of the best sound speed calculated a few times during this loop
        %show this value every iteration slows down the simulation
        disp(strcat('Error=',num2str(dist_from_real_point,4),'m, Sound Speed=',num2str(soundspeed,5),'m/s'));
        pause(0.01);
    end
    eixo_x=[eixo_x, soundspeed];
    eixo_y=[eixo_y, dist_from_real_point];
    
    soundspeed=soundspeed+0.01;
end

    pause(0.1);
    disp(strcat('End of calibration, actual sound speed [', num2str(best_values(2),5),' m/s]'));
    sound_speed = best_values(2);

    
% Clear the variables used only for calibration
% clear best_values dist_from_real_point soundspeed time_measurements;
% clear time_index num_calib_data text41_box_value;
% clear x_calib y_calib z_calib;