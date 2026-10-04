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
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSx(i),'r*', 'MarkerSize',5);end;end %plot SILA as *
    xlabel('tempo[s]','FontSize',fontsize); ylabel('eixo X[m]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('posição estimada pelo Filtro de Kalman','posição calculada pelo SILA');
    lgd.FontSize=fontsize;
    %lgd.Location='southeast';
    lgd.Position=[0.67 0.73 0.2727 0.0617];
    hold off;

subplot(3,1,2);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Py_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,LSy(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSy(i),'r*', 'MarkerSize',5);end;end
    xlabel('tempo [s]','FontSize',fontsize); ylabel('eixo Y [m]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('posição estimada pelo Filtro de Kalman','posição calculada pelo SILA');
    lgd.FontSize=fontsize;
    lgd.Position=[0.67 0.56 0.2727 0.0617];
    hold off;
    
subplot(3,1,3);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,psi_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,psi_comp(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, psi_comp(i),'r*', 'MarkerSize',5);end;end
    xlabel('tempo [s]','FontSize',fontsize); ylabel('ângulo de guinada [º]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('ângulo estimado pelo Filtro de Kalman','ângulo medido pela Bússola Digital');
    lgd.FontSize=fontsize;
    %lgd.Location='southeast';
    lgd.Position=[0.67 0.13 0.2673 0.0617];
    hold off;
    
pause(1);
output_img = 'estados_estimados_completo';
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
% output_img = 'bias_estimados_completo';
% print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);