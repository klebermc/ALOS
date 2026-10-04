clear
hold on
iter=1;
figure;

pontos=[
0.195,  0.200;
0.065,  1.490;
0.245,  2.870;
1.245,  0.205;
1.250,  1.500;
1.260,  2.870;
2.355,  0.195;
2.360,  1.500;
2.350,  2.870
];

alturas=[0.06 0.54 1.04 1.54];
close all;

% disp('_______X______|_______Y______|_______Z______')
% disp('min max med dp|min max med dp|min max med dp')
% table=[];

for h=0:3
    
    %% Responsável por iniciar a figura
    figure('pos',[400 50 600 600])
    axis equal;
    axis([-0.75 3.25 -0.5 3.5]);
    grid on;
    hold on;
    
    %Linhas das extremidades.
    plot([0 0],[0 3],'k','LineWidth',2);
    plot([0 2.5],[3 3],'k','LineWidth',2);
    plot([0 2.5],[0 0],'k','LineWidth',2);
    plot([2.5 2.5],[0 3],'k','LineWidth',2);
    
    for p=1:9
        file2open = strcat('ponto',num2str(p),'_altura',num2str(h),'.mat');
        load(file2open);
        
        %% Essa parte plota o dp das medidas (não do erro) para cada eixo
%         disp(file2open);
%         disp([std(module_position_lms(:,1)) std(module_position_lms(:,2)) std(module_position_lms(:,3))]*100)
%         disp([std(module_position_taylor(:,1)) std(module_position_taylor(:,2)) std(module_position_taylor(:,3))])
%         disp([std(module_position_aml(:,1)) std(module_position_aml(:,2)) std(module_position_aml(:,3))]*100)
%         if h==1 ; subplot(3,3,p); histogram(module_position_taylor(:,3)); end
        
        %% Essa parte é para gerar os dados da tabela, min max avg dp do erro para cada eixo
%         errox=module_position_taylor(:,1) - pontos(p,1);
%         erroy=module_position_taylor(:,2) - pontos(p,2);
%         erroz=module_position_taylor(:,3) - alturas(h+1);
%         fprintf('%.3f & %d & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f\n',alturas(h+1),p,min(errox),max(errox),mean(errox),std(errox),min(erroy),max(erroy),mean(erroy),std(erroy),min(erroz),max(erroz),mean(erroz),std(erroz))
%         table=[table; min(errox),max(errox),mean(errox),std(errox),min(erroy),max(erroy),mean(erroy),std(erroy),min(erroz),max(erroz),mean(erroz),std(erroz)];

        %% Essa parte faz o plot da figura de erros
        for i=1:max(size(module_position_taylor))
            dist(i) = sqrt( (module_position_taylor(i,1)-pontos(p,1))^2 + (module_position_taylor(i,2)-pontos(p,2))^2 + (module_position_taylor(i,3)-alturas(h+1))^2 );
        end
        
        viscircles([pontos(p,1) pontos(p,2)],mean(dist), 'Color', [min(1,mean(dist)*2) (1-min(1,mean(dist)*2)) 0], 'LineWidth', 2);
        plot(pontos(p,1), pontos(p,2), '.', 'MarkerSize',20,'Color', 'k')
        text(pontos(p,1)+0.05,pontos(p,2),strcat('d=',num2str(mean(dist),3),'m'),'FontSize',12);
        
        clear('availability_history','module_position_lms','module_position_aml','module_position_taylor', 'measured_time_history', 'sound_speed', 'stationary_modules_positions');
    end
    
    %% Responsável por gerar as figuras, salvar o png e enviara a para a pasta do latex
    xlabel('X[m]','FontSize',14); ylabel('Y[m]','FontSize',14);
    title(strcat('Distância média entre p. calculada e p. real / altura=',num2str(alturas(h+1),3),'cm'))

    pause(1);
    output_img = strcat('v2_erro_altura_',num2str(int8(h)));
    print(output_img,'-dpng');
    % command = ['mv', ' ' , output_img, '.png ./figures/'];
    % system(command);
    close all;
    pause(1);
    
end

% mean(table)