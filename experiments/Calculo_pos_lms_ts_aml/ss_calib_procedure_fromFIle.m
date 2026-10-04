clear;

load('../Medidas_Receptor_Parado_V2/ponto5_altura0.mat');

n_emitters = 8;

stationary_modules_positions = [0.000  0.05  2.48;
                                1.285  0.05  2.96;
                                2.535  0.05  2.48; 
                                2.915  1.60  1.99; 
                                2.570  3.05  2.48; 
                                1.260  3.05  2.96;
                                -0.05  2.90  2.48;
                                -0.47  1.52  1.77];

%Here is the calibration procedure
time_measurements=measured_time_history(1:20,:)*(1e-6);

%The position of the module for calibration
x_calib = 1.25;
y_calib = 1.50;
z_calib = 0.06; %ground

%Iterate, testing different values for the the sound speed until I found the one with smaller error.
soundspeed = 200;
best_values=[100 soundspeed];
distance_measurements=[];

eixo_x=[];
eixo_y=[];

while soundspeed<400
    
    for i=1:size(time_measurements,2)
        distance_measurements(i) = median(time_measurements(:,i)) * soundspeed;
    end
    
    [module_x,module_y,module_z] = LMS(stationary_modules_positions, distance_measurements); filename='vel_calib_ls';
%     [module_x,module_y,module_z] = taylor_series(stationary_modules_positions, distance_measurements, [module_x,module_y,module_z] ); filename='vel_calib_ls_ts';
    
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
plot(eixo_x,eixo_y,'k','LineWidth',1.5); grid on;
xlabel('velocidade do som [m/s]','FontSize',14); ylabel('erro pos. calculada e pos. real [m]','FontSize',14);
print(filename,'-dpng')

for i=1:8
    dist = sqrt( (stationary_modules_positions(i,1)-1.25)^2 + (stationary_modules_positions(i,2)-1.50)^2 + (stationary_modules_positions(i,3)-0.06)^2 );
    time = median(measured_time_history(:,i))*(1e-6);
    vel(i)=dist/time;
end

disp( strcat('media vel =[', num2str(mean(vel)),'] mediana vel =[', num2str(median(vel)), ']'));