%comeco=11; final=690;
%comeco=700; final=1637;
comeco=1650; final=2800;
%comeco=2820; final=3570;

for i=1:length(psi_est)
    psi_est2(i) = psi_est(i);
    if psi_est(i)>180
        psi_est2(i) = psi_est(i)-360;
    end
    if psi_est(i)<-180
        psi_est2(i) = psi_est(i)+360;
    end
end

close all;

figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Px_est(comeco:final), 'LineWidth', 1.5);
%     plot((count(comeco:final)-count(comeco))/10,LSx(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSx(i),'r*', 'MarkerSize',5);end;end %plot SILA as *
%     xlabel('tempo[s]','FontSize',14); 
    ylabel('eixo X[m]','FontSize',14);
    %title('Posição no eixo X');
    lgd=legend('posição estimada pelo Filtro de Kalman','posição calculada pelo SILA');
    lgd.FontSize=11;
    lgd.Position=[0.74 0.855 0.25 0.0588];
    hold off;

subplot(3,1,2);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Py_est(comeco:final), 'LineWidth', 1.5);
%     plot((count(comeco:final)-count(comeco))/10,LSy(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSy(i),'r*', 'MarkerSize',5);end;end
%     xlabel('tempo [s]','FontSize',14); 
    ylabel('eixo Y [m]','FontSize',14);
    %title('Posição no eixo X');
    lgd=legend('posição estimada pelo Filtro de Kalman','posição calculada pelo SILA');
    lgd.FontSize=11;
    lgd.Position=[0.74 0.555 0.25 0.0588];
    hold off;
    
subplot(3,1,3);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,psi_est2(comeco:final), 'LineWidth', 1.5);
%     plot((count(comeco:final)-count(comeco))/10,psi_comp(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, psi_comp(i),'r*', 'MarkerSize',5);end;end
    xlabel('tempo [s]','FontSize',14); ylabel('ângulo de guinada [º]','FontSize',14);
    %title('Posição no eixo X');
    lgd=legend('ângulo estimado pelo Filtro de Kalman','ângulo medido pela Bússola Digital');
    lgd.FontSize=11;
    lgd.Position=[0.75 0.2470 0.242 0.0588];
    hold off;

% draw the lines
annotation('line',[0.235 0.235],[0.1 0.95],'color','k', 'LineWidth', 1.5);
annotation('line',[0.36 0.36],[0.1 0.95],'color','k', 'LineWidth', 1.5);
annotation('line',[0.50 0.50],[0.1 0.95],'color','k', 'LineWidth', 1.5);
annotation('line',[0.63 0.63],[0.1 0.95],'color','k', 'LineWidth', 1.5);

%the fault label
annotation('textbox',[.275 .36 .0 .63],'String','Falha','FitBoxToText','on','FontSize',11);
annotation('doublearrow',[0.235 0.36],[0.95 0.95])
annotation('textbox',[.54 .36 .0 .63],'String','Falha','FitBoxToText','on','FontSize',11);
annotation('doublearrow',[0.50 0.63],[0.95 0.95])

pause(1);
output_img = 'estados_estimados_perdaSLB';
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
%     axis([0 (count(final)-count(comeco))/10 -0.1 0.1])
%     plot((count(comeco:final)-count(comeco))/10,biasax(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo[s]','FontSize',14); ylabel('bias ax [m/s^2]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
% 
% subplot(3,1,2);
%     hold on;
%     grid on;
%     axis([0 (count(final)-count(comeco))/10 -0.1 0.1])
%     plot((count(comeco:final)-count(comeco))/10,biasay(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('bias ay [m/s^2]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
%     
% subplot(3,1,3);
%     hold on;
%     grid on;
%     axis([0 (count(final)-count(comeco))/10 -0.1 0.1])
%     plot((count(comeco:final)-count(comeco))/10,biaswz(comeco:final), 'LineWidth', 1.5);
%     xlabel('tempo [s]','FontSize',14); ylabel('bias w/z [rad/s]','FontSize',14);
%     %title('Posição no eixo X');
%     hold off;
%     
% pause(1);
% output_img = 'bias_estimados_perdaSLB';
% print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);