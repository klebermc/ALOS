clear; close all;
% load('../Medidas_Receptor_Parado_V2/ponto9_altura0.mat')

%o ponto de avaliação a se analisar os dados
p=9;

disp('medidas em cm');
disp(strcat('ponto=',num2str(int8(p))));
load(strcat('../Medidas_Receptor_Parado_V2/ponto',num2str(int8(p)), '_altura0.mat'));


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


%% LMS
% figure
% close all;
plot(module_position_lms,'DisplayName','module_position_lms')
grid on
hold on
plot([1 max(size(module_position_lms))], [pontos(p,1) pontos(p,1)])
plot([1 max(size(module_position_lms))], [pontos(p,2) pontos(p,2)])
plot([1 max(size(module_position_lms))], [0.06 0.06])
axis([1 250 -1.5 3.5])
lgd=legend('p_x calc','p_y calc','p_z calc', 'p_x real','p_y real','p_z real');
lgd.Location='east';
xlabel('tempo [s]')
ylabel('posição [m]')
print(strcat('p',num2str(int8(p)),'_ls'),'-dpng')

errox_lms=module_position_lms(:,1) - pontos(p,1);
erroy_lms=module_position_lms(:,2) - pontos(p,2);
erroz_lms=module_position_lms(:,3) - 0.06;
e=[];

for i=1:200
e(i)=sqrt((module_position_lms(i,1)-pontos(p,1))^2 +(module_position_lms(i,2)-pontos(p,2))^2 +(module_position_lms(i,3)-0.06)^2);
end
% disp(strcat('erro medio entre LMS e real: ', num2str(mean(e))));
fprintf('%.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f \n', mean(errox_lms)*100,std(errox_lms)*100,mean(erroy_lms)*100,std(erroy_lms)*100,mean(erroz_lms)*100,std(erroz_lms)*100,mean(e)*100)
pause(1);

%% TS
% figure
close all;
plot(module_position_taylor,'DisplayName','module_position_taylor')
%axis([1 200 0 0.5])
grid on
hold on
plot([1 max(size(module_position_lms))], [pontos(p,1) pontos(p,1)])
plot([1 max(size(module_position_lms))], [pontos(p,2) pontos(p,2)])
plot([1 max(size(module_position_lms))], [0.06 0.06])
axis([1 250 -1.5 3.5])
lgd=legend('p_x calc','p_y calc','p_z calc', 'p_x real','p_y real','p_z real');
lgd.Location='east';
xlabel('tempo [s]')
ylabel('posição [m]')
print(strcat('p',num2str(int8(p)),'_ts'),'-dpng')

errox_taylor=module_position_taylor(:,1) - pontos(p,1);
erroy_taylor=module_position_taylor(:,2) - pontos(p,2);
erroz_taylor=module_position_taylor(:,3) - 0.06;
e=[];

for i=1:200
e(i)=sqrt((module_position_taylor(i,1)-pontos(p,1))^2 +(module_position_taylor(i,2)-pontos(p,2))^2 +(module_position_taylor(i,3)-0.06)^2);
end
% disp(strcat('erro medio entre TS e real: ', num2str(mean(e))));
fprintf('%.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f \n', mean(errox_taylor)*100,std(errox_taylor)*100,mean(erroy_taylor)*100,std(erroy_taylor)*100,mean(erroz_taylor)*100,std(erroz_taylor)*100,mean(e)*100)
pause(1);

%% AML
% figure
close all;
plot(module_position_aml,'DisplayName','module_position_aml')
%axis([1 200 0 0.5])
grid on
hold on
plot([1 max(size(module_position_lms))], [pontos(p,1) pontos(p,1)])
plot([1 max(size(module_position_lms))], [pontos(p,2) pontos(p,2)])
plot([1 max(size(module_position_lms))], [0.06 0.06])
axis([1 250 -1.5 3.5])
lgd=legend('p_x calc','p_y calc','p_z calc', 'p_x real','p_y real','p_z real');
lgd.Location='east';
xlabel('tempo [s]')
ylabel('posição [m]')
print(strcat('p',num2str(int8(p)),'_aml'),'-dpng')

errox_aml=module_position_aml(:,1) - pontos(p,1);
erroy_aml=module_position_aml(:,2) - pontos(p,2);
erroz_aml=module_position_aml(1:150,3) - 0.06;
e=[];

for i=1:150
e(i)=sqrt((module_position_aml(i,1)-pontos(p,1))^2 +(module_position_aml(i,2)-pontos(p,2))^2 +(module_position_aml(i,3)-0.06)^2);
end
% disp(strcat('erro medio entre AML e real: ', num2str(mean(e))));
fprintf('%.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f \n', mean(errox_aml)*100,std(errox_aml)*100,mean(erroy_aml)*100,std(erroy_aml)*100,mean(erroz_aml)*100,std(erroz_aml)*100,mean(e)*100)
