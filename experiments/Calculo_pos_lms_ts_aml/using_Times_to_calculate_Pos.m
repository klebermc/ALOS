clear;
close all;
load('../Medidas_Receptor_Parado_V2/ponto5_altura0.mat');
p=5;

n_emitters = 8;

% stationary_modules_positions = [0.000  0.05  2.48;
%                                 1.285  0.05  2.96;
%                                 2.535  0.05  2.48; 
%                                 2.915  1.60  1.99; 
%                                 2.570  3.05  2.48; 
%                                 1.260  3.05  2.96;
%                                 -0.05  2.90  2.48;
%                                 -0.47  1.52  1.77];

pontos=[
0.195,  0.200;
0.065,  1.490;
0.245,  2.870;
1.245,  0.205;
1.250,  1.500;
1.260,  2.870;
2.355,  0.195;
2.360,  1.500;
2.350,  2.870];

%Iterate, testing different values for the the sound speed until I found the one with smaller error.
soundspeed = 337.51; filename='pos_xyz_calib_lms';
% soundspeed = 335.39; filename='pos_xyz_calib_lms_ts';

distance_measurements=[];

posicao_modulo=[];
tempo_comp=[];

for i=1:200
    available_modules=[];
    available_measurements=[];
    for j=1:8
        if  measured_time_history(i,j)>100 && measured_time_history(i,j)<15000
            %only consider as available, modules that the
            %measurement appears in the message, and that are
            %less then 15ms (max time waited to be out of range)
            available_modules = [available_modules; stationary_modules_positions(j,:)];
            available_measurements = [available_measurements, measured_time_history(i,j) * soundspeed * (1e-6)];
        end                            
    end
    tstart=tic;
    [module_x,module_y,module_z] = LMS(available_modules, available_measurements);
%     [module_x,module_y,module_z] = taylor_series(available_modules, available_measurements, [module_x,module_y,module_z] ); 
    telapsed=toc(tstart);
    tempo_comp=[tempo_comp, telapsed];
    
    posicao_modulo=[posicao_modulo; module_x,module_y,module_z];
    
end

errox=posicao_modulo(:,1) - pontos(p,1);
erroy=posicao_modulo(:,2) - pontos(p,2);
erroz=posicao_modulo(:,3) - 0.06;

disp('Media do erro')
[mean(errox) mean(erroy) mean(erroz)]*100

pause(0.1);
plot(posicao_modulo); 
grid on;
hold on;

plot([1 max(size(module_position_lms))], [pontos(p,1) pontos(p,1)]) %posicao real x
plot([1 max(size(module_position_lms))], [pontos(p,2) pontos(p,2)]) %posicao real y
plot([1 max(size(module_position_lms))], [0.06 0.06]) %posicao real z

axis([1 max(size(posicao_modulo)) 0 1.7])

xlabel('tempo [s]'); ylabel('posição do módulo receptor [m]');
legend('p_x - calc','p_y - calc','p_z - calc','p_x - real','p_y - real','p_z - real', 'Location', 'SouthEast');
% print(filename,'-dpng')

for i=1:max(size(posicao_modulo))
    erro(i) = sqrt((posicao_modulo(i,1)-pontos(p,1))^2 +(posicao_modulo(i,2)-pontos(p,2))^2 +(posicao_modulo(i,3)-0.06)^2);
end

disp( strcat(filename, '=> MEDIA erro p.real p.calc =[', num2str(mean(erro)*100),' cm] MEDIANA erro p.real p.calc =[', num2str(median(erro)*100), 'cm]'));