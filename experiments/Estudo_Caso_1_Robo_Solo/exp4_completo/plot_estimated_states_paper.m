comeco=3740;
final=4780;
close all;
fontsize=12;


figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Px_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,LSx(comeco:final), 'LineWidth', 1.5);
    for i=comeco:comeco+10;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSx(i),'r*', 'MarkerSize',5);end;end %plot SILA as *
    plot([  0 178]/10,[2.10 2.10], '--k','LineWidth', 0.8);%waypoint1
    xlabel('time[s]','FontSize',fontsize); ylabel('Position X[m]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('Navigation System estimated pos.','ALOS estimated pos.','Next Waypoint');
    lgd.FontSize=fontsize;
    %lgd.Location='southeast';
    lgd.Position=[0.4 0.85 0.25 0.0617];
    
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSx(i),'r*', 'MarkerSize',5,'HandleVisibility','off');end;end %plot SILA as *
    
    plot([178 178]/10,[2.10 1.25], '--k','LineWidth', 0.8);%waypoint1to2
    plot([178 252]/10,[1.25 1.25], '--k','LineWidth', 0.8);%waypoint2
    plot([252 252]/10,[1.25 0.50], '--k','LineWidth', 0.8);%waypoint2to3
    plot([252 337]/10,[0.50 0.50], '--k','LineWidth', 0.8);%waypoint3
    plot([337 337]/10,[0.50 1.25], '--k','LineWidth', 0.8);%waypoint3to4
    plot([337 476]/10,[1.25 1.25], '--k','LineWidth', 0.8);%waypoint4
    plot([476 476]/10,[1.25 0.50], '--k','LineWidth', 0.8);%waypoint4to5
    plot([476 595]/10,[0.50 0.50], '--k','LineWidth', 0.8);%waypoint5
    plot([595 595]/10,[0.50 1.25], '--k','LineWidth', 0.8);%waypoint5to6
    plot([595 704]/10,[1.25 1.25], '--k','LineWidth', 0.8);%waypoint6
    plot([704 704]/10,[1.25 2.10], '--k','LineWidth', 0.8);%waypoint6to7
    plot([704 803]/10,[2.10 2.10], '--k','LineWidth', 0.8);%waypoint7
    plot([803 803]/10,[2.10 1.25], '--k','LineWidth', 0.8);%waypoint7to8
    plot([803 945]/10,[1.25 1.25], '--k','LineWidth', 0.8);%waypoint8
    
    hold off;
subplot(3,1,2);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Py_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,LSy(comeco:final), 'LineWidth', 1.5);
    for i=comeco:comeco+10;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSy(i),'r*', 'MarkerSize',5);end;end
    plot([  0 178]/10,[2.40 2.40], '--k','LineWidth', 0.8);%waypoint1
    xlabel('time [s]','FontSize',fontsize); ylabel('Position Y [m]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('Navigation System estimated pos.','ALOS estimated pos.','Next Waypoint');
    lgd.FontSize=fontsize;
    lgd.Position=[0.5 0.55 0.25 0.0617];
    
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSy(i),'r*', 'MarkerSize',5);end;end
    
    plot([178 178]/10,[2.40 2.40], '--k','LineWidth', 0.8);%waypoint1to2
    plot([178 252]/10,[2.40 2.40], '--k','LineWidth', 0.8);%waypoint2
    plot([252 252]/10,[2.40 2.40], '--k','LineWidth', 0.8);%waypoint2to3
    plot([252 337]/10,[2.40 2.40], '--k','LineWidth', 0.8);%waypoint3
    plot([337 337]/10,[2.40 1.50], '--k','LineWidth', 0.8);%waypoint3to4
    plot([337 476]/10,[1.50 1.50], '--k','LineWidth', 0.8);%waypoint4
    plot([476 476]/10,[1.50 0.70], '--k','LineWidth', 0.8);%waypoint4to5
    plot([476 595]/10,[0.70 0.70], '--k','LineWidth', 0.8);%waypoint5
    plot([595 595]/10,[0.70 0.70], '--k','LineWidth', 0.8);%waypoint5to6
    plot([595 704]/10,[0.70 0.70], '--k','LineWidth', 0.8);%waypoint6
    plot([704 704]/10,[0.70 0.70], '--k','LineWidth', 0.8);%waypoint6to7
    plot([704 803]/10,[0.70 0.70], '--k','LineWidth', 0.8);%waypoint7
    plot([803 803]/10,[0.70 1.50], '--k','LineWidth', 0.8);%waypoint7to8
    plot([803 945]/10,[1.50 1.50], '--k','LineWidth', 0.8);%waypoint8
        
    hold off;
    
subplot(3,1,3);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,psi_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,psi_comp(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, psi_comp(i),'r*', 'MarkerSize',5);end;end
    xlabel('time [s]','FontSize',fontsize); ylabel('Heading angle [º]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('Navigation System estimated angle','Digital compass measured angle');
    lgd.FontSize=fontsize;
    %lgd.Location='southeast';
    lgd.Position=[0.65 0.13 0.25 0.0617];
    hold off;
    
pause(1);
output_img = 'states_estimated_fullsys';
print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);

% % Essa parte plota o erro em cada um dos eixos, não acho que esteja certo, pq estou comparando os
% estados estimados com a última SL e Bussola
% close all;
% figure('units','normalized','outerposition',[0 0 1 1])
% subplot(3,1,1);
%     hold on;
%     grid on;
%     axis([0 95 -0.8 0.8])
%     plot((count(comeco:final)-count(comeco))/10,(Px_est(comeco:final)-LSx(comeco:final)), 'LineWidth', 1.5);
%     xlabel('tempo[s]','FontSize',14); ylabel('erro em X[m]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
% 
% subplot(3,1,2);
%     hold on;
%     grid on;
%     axis([0 95 -0.8 0.8])
%     plot((count(comeco:final)-count(comeco))/10,(Py_est(comeco:final)-LSy(comeco:final)), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('erro em Y [m]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
%     
% subplot(3,1,3);
%     hold on;
%     grid on;
%     axis([0 95 -45 45])
%     plot((count(comeco:final)-count(comeco))/10,hampel(psi_est(comeco:final)-psi_comp(comeco:final)), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('erro de guinada [º]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;

% % Essa parte imprime os bias
% close all;
% figure('units','normalized','outerposition',[0 0 1 1])
% subplot(3,1,1);
%     hold on;
%     grid on;
%     %axis([0 95 -0.8 0.8])
%     plot((count(comeco:final)-count(comeco))/10,biasax(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo[s]','FontSize',14); ylabel('bias ax [m/s^2]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
% 
% subplot(3,1,2);
%     hold on;
%     grid on;
%     %axis([0 95 -0.8 0.8])
%     plot((count(comeco:final)-count(comeco))/10,biasay(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('bias ay [m/s^2]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
%     
% subplot(3,1,3);
%     hold on;
%     grid on;
%     %axis([0 95 -45 45])
%     plot((count(comeco:final)-count(comeco))/10,biaswz(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('bias w/z [rad/s]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
%     
% pause(1);
% output_img = 'bias_estimado_completo';
% print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);