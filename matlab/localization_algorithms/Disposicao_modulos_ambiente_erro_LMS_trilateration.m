% Esse algoritmo foi usado para, dado um ponto virtual a área de testes, tentar decidir qual a
% configuração de módulos emissores no ambiente que gera o menor erro final.

% ***************** Algoritmo para fazer a decisão: ********************
% 1- Escolhe-se um ponto virtual (P1) de localização conhecida na área de testes;
% 2- Calcula-se a distância entre esse ponto e os módulos emissores (valor ideal de medida do sistema de localização), dn = P1 dist En, onde n é o número do ID do módulo emissor;
% 3- Acrescenta-se nessa media um ruido aleatório dado por uma distribuição normal, dn = dn + ruido;
% 5- Calcula-se, utilizando o algoritmo LMS, a posição do ponto virtual utilizando as medidas ruidosas, P2 = LMS(d=[d1,...dn], Localização dos módulos emissores);
% 6- Encontra-se a o erro de medida, definido como a distância euclidiana entre o ponto calculado e o ponto virtual original, e = dist_eucl(P1, P2);
% 7- Repete o algoritmo a partir do passo 3 100 vezes, e obtem-se uma média para os valores de e

close all;
clear;

error=[];
labels=['a','b','c','d','e','f', 'g','h','i','j','k','l'];

positions = [0.00  0.00  2.50;
           1.25  0.00  2.00;
           1.25  0.00  3.00;
           2.50  0.00  2.50;
           2.90  1.60  2.00;
           2.90  1.60  3.00;
           2.50  3.00  2.50;
           1.25  3.00  2.00;
           1.25  3.00  3.00;
           0.00  2.90  2.50;
          -0.50  1.50  1.80;
          -0.50  1.50  3.00
                 ];

%para simular, apenas descomente a configuração que deseja estudar, o código vai: gera o gráfico,
%colocar as labels, salvar um png com nome da configuração, calcular erro médio e erro máximo entre
%o ponto virtual escolhido e os pontos calculados a partir de medidas ruidosas


% %posição dos módulos testes
%%%4+1L
% config_name='41L';
% stationary_modules_positions = [0.00  0.00  2.50;
%                                 2.50  0.00  2.50; 
%                                 2.90  1.60  2.00; 
%                                 2.50  3.00  2.50;
%                                 0.00  2.90  2.50
%                                 ];

%%%4+2L
% config_name='42L';
% stationary_modules_positions = [0.00  0.00  2.50;
%                                 2.50  0.00  2.50; 
%                                 2.90  1.60  2.00; 
%                                 2.50  3.00  2.50;
%                                 0.00  2.90  2.50;
%                                -0.50  1.50  1.80
%                                 ];

%%%4+2H
% config_name='42H';
% stationary_modules_positions = [0.00  0.00  2.50;
%                                 2.50  0.00  2.50; 
%                                 2.90  1.60  3.00;
%                                 2.50  3.00  2.50;
%                                 0.00  2.90  2.50;
%                                -0.50  1.50  3.00
%                                 ];

%%%4+4L
% config_name='44L';
% stationary_modules_positions = [0.00  0.00  2.50;
%                                 1.25  0.00  2.00;
%                                 2.50  0.00  2.50; 
%                                 2.90  1.60  2.00; 
%                                 2.50  3.00  2.50;
%                                 1.25  3.00  2.00;
%                                 0.00  2.90  2.50;
%                                -0.50  1.50  1.80
%                                 ];

%%%4+4H
% config_name='44H';
% stationary_modules_positions = [0.00  0.00  2.50;
%                                 1.25  0.00  3.00;
%                                 2.50  0.00  2.50; 
%                                 2.90  1.60  3.00;
%                                 2.50  3.00  2.50;
%                                 1.25  3.00  3.00;
%                                 0.00  2.90  2.50;
%                                -0.50  1.50  3.00
%                                 ];

%%%4+2H+2L
config_name='42H2L';
stationary_modules_positions = [0.00  0.00  2.50;
                                1.25  0.00  3.00;
                                2.50  0.00  2.50; 
                                2.90  1.60  2.00; 
                                2.50  3.00  2.50;
                                1.25  3.00  3.00;
                                0.00  2.90  2.50;
                               -0.50  1.50  1.80
                                ];
                            
num_statinary_modules = size(stationary_modules_positions,1);

%ponto escolhido para teste
x=1.25;
y=1.5 ;
z=0.5 ;

measurements_m=[];
measurements_new_m=[];

%inicia o plot da figura
figure;
plot3(stationary_modules_positions(1,1),stationary_modules_positions(1,2),stationary_modules_positions(1,3),'rx');
hold on;
plot3([stationary_modules_positions(1,1) stationary_modules_positions(1,1)],[stationary_modules_positions(1,2) stationary_modules_positions(1,2)],[0 stationary_modules_positions(1,3)],'k','LineWidth',1.5);

i=1;
for j = 1:max(size(positions))
    if stationary_modules_positions(i,1)==positions(j,1) && stationary_modules_positions(i,2)==positions(j,2) && stationary_modules_positions(i,3)==positions(j,3)
    text(stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3)+0.25,labels(j),'FontSize',12);
    end
end
    
for i=2:num_statinary_modules
    plot3(stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3),'rx');
    if stationary_modules_positions(i,3) > 2.6
        plot3([stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'g','LineWidth',1.5);
    end
    if stationary_modules_positions(i,3) > 2.4 && stationary_modules_positions(i,3) < 2.6
        plot3([stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'k','LineWidth',1.5);
    end
    if stationary_modules_positions(i,3) < 2.4
        plot3([stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'r','LineWidth',1.5);
    end
    
    for j = 1:max(size(positions))
        if stationary_modules_positions(i,1)==positions(j,1) && stationary_modules_positions(i,2)==positions(j,2) && stationary_modules_positions(i,3)==positions(j,3)
        text(stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3)+0.25,labels(j),'FontSize',12);
        end
    end

end
axis([min(positions(:,1)) max(positions(:,1)) min(positions(:,2)) max(positions(:,2)) 0 max(positions(:,3))+0.5]); %The axis limits
xlabel('X[metros]');
ylabel('Y[metros]');
zlabel('Z[metros]');

grid on;
plot3(x,y,z,'b*');
plot3([x x],[y y],[0 z],'b','LineWidth',1.5);
text(x-0.15,y+0.2,z,'Pv','FontSize',12);
print(config_name,'-dpng');

% axis([0 3 0 3 0 3]);

%Calcula as distâncias entre o ponto escolhido e posição dos n módulos
for i=1:num_statinary_modules
    m(i) = sqrt( (x-stationary_modules_positions(i,1))^2 + (y-stationary_modules_positions(i,2))^2 + (z-stationary_modules_positions(i,3))^2 );
end
    
for i=1:100
    
    dp = 5; %desvio padrao em centímetros
    distance_readings=[];
    for i=1:num_statinary_modules
        new_m(i) = m(i) + (dp/100)*randn;  %adicionando ruido às leituras de medidas
        distance_readings = [distance_readings,new_m(i)];
    end
    
    measurements_m = [measurements_m;m];
    measurements_new_m=[measurements_new_m;new_m];
    
%     stationary_modules_positions
%     distance_readings
    
    [module_x,module_y,module_z] = trilateration(stationary_modules_positions, distance_readings);
    
%     plot3(module_x,module_y,module_z,'bo');
%     plot3([x module_x],[y module_y], [z module_z],'k');
    
%     disp('--------------------')
%     disp(distance_readings)
%     disp([x,y,z])
%     disp([module_x,module_y,module_z])
%     disp(sqrt( (x-module_x)^2 + (y-module_y)^2 + (z-module_z)^2 ))
%     disp('--------------------')
%     pause
    error = [error, sqrt( (x-module_x)^2 + (y-module_y)^2 + (z-module_z)^2 )];
end
hold off;

disp(config_name)
disp(strcat('media=',num2str(mean(error))));
disp(strcat('max error = ', num2str(max(error))));

% figure;
% subplot(3,1,1);
% plot(error);
% grid on;
% title('Position Measurement Error');
% ylabel('error (meters)');
% legend(strcat('max error = ', num2str(max(error))));
% 
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