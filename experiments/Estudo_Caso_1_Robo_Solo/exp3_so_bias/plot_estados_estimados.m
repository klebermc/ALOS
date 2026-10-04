comeco=13;
final=313;
close all;
fontsize=12;

figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1);
    hold on;
    grid on;
    plot((count(comeco:final)-count(comeco))/10,Px_est(comeco:final), 'LineWidth', 1.5);
    %plot((count(comeco:final)-count(comeco))/10,LSx(comeco:final), 'LineWidth', 1.5);
    for i=comeco:final;if X_sig(i)==1; plot((count(i)-count(comeco))/10, LSx(i),'r*', 'MarkerSize',5);end;end
    xlabel('tempo[s]','FontSize',fontsize); ylabel('eixo X[m]','FontSize',fontsize);
    %title('Posição no eixo X');
    lgd=legend('posição estimada pelo Filtro de Kalman','posição calculada pelo SILA');
    lgd.FontSize=fontsize;
    lgd.Position=[0.72 0.855 0.2727 0.0643];
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
    lgd.Position=[0.72 0.4373 0.2727 0.0617];
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
    lgd.Position=[0.725 0.2408 0.2673 0.0643];
    hold off;
    
pause(1);
output_img = 'estados_estimados_so_bias';
print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);