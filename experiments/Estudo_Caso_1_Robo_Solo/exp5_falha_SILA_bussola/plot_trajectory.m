% pontos=[
% 0.195,  0.200;
% 0.065,  1.490;
% 0.245,  2.870;
% 1.245,  0.205;
% 1.250,  1.500;
% 1.260,  2.870;
% 2.355,  0.195;
% 2.360,  1.500;
% 2.350,  2.870];

WAYPOINTS = [   
2.10 2.40 0; %p9
1.25 2.40 0; %p6
0.50 2.40 0; %p3
1.25 1.50 0; %p5
0.50 0.70 0; %p1
1.25 0.70 0; %p4
2.10 0.70 0; %p7
1.25 1.50 0; %p5
            ];
        
% Responsável por iniciar a figura
close all
figure('pos',[400 50 600 600])
%axis equal;
axis([-0.5 3 -0.5 3.5]);
grid on;
hold on;

%Linhas das extremidades.
plot([-0.02     -0.02],[-0.02 3.02],'k','LineWidth',2);
plot([-0.02   2.5],[3.02 3.02],'k','LineWidth',2);
plot([2.5 2.5],[-0.02 3.02],'k','LineWidth',2);
plot([-0.02   2.5],[-0.02 -0.02],'k','LineWidth',2);

% plot(pontos(:,1), pontos(:,2), '.', 'MarkerSize',20,'Color', 'k')
for p=1:7
    plot(WAYPOINTS(p,1), WAYPOINTS(p,2), '.', 'MarkerSize',20,'Color', 'k')
    viscircles([WAYPOINTS(p,1) WAYPOINTS(p,2)],0.25, 'Color', 'k', 'LineWidth', 1);
end

hold on;

%testes de 1 a 4
%comeco=11; final=690;
%comeco=700; final=1637;
comeco=1650; final=2800;
%comeco=2820; final=3570;

for i=comeco:final
    %p1 = plot([min(Px_est(i-1),2.5) min(Px_est(i),2.5)], [Py_est(i-1) Py_est(i)], 'b');
    %p2 = plot(LSx(i), LSy(i), 'o', 'MarkerSize',5,'Color', 'r');
    p1 = plot([LSx(i-1) LSx(i)], [LSy(i-1) LSy(i)], 'r');
%     title(strcat('i=',num2str(int16(i)),'/', num2str(int16(final))));  pause(0.1);
end

xlabel('X[m]','FontSize',14); ylabel('Y[m]','FontSize',14);
title('Trajetória estimada - perda momentânia de SL+B');
%legend([p1,p2],'posição estimada Filtro de Kalman','posição calculada SL');
legend([p1],'trajetória calculada pelo sistema de localização');

pause(1);
output_img = 'trajetoria_perdaSLB';
print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);
