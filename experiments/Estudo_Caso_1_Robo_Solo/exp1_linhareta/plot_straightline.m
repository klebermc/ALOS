%import .txt first

fontsize=15;
linewidth=1.5;
markersize=8;
tempo=count/10;

%calculating the error in estimation
% tempo_real=0:0.1:21;
% posX_real=0.50:((1.97-0.50)/length(tempo_real)):1.97;
% posY_real=0.70:((2.08-0.70)/length(tempo_real)):2.08;
% SE_X=zeros(1,211);
% SE_Y=zeros(1,211);
% j=1;
% for i=496:706
%     SE_X(i-495) = (posX_real(i-495)-Px_est(i))^2;
%     SE_Y(i-495) = (posY_real(i-495)-Py_est(i))^2;
%     SE_psi(i-495)= (43.19-psi_est(i))^2;
%     if X_sig(i)==1
%         SE_SILA_X(j) = (posX_real(i-495)-LSx(i))^2;
%         SE_SILA_Y(j) = (posY_real(i-495)-LSy(i))^2;
%         SE_psicomp(j)= (43.19-psi_comp(i))^2;
%         j=j+1;
%     end
% end
% RMSE_X=sqrt(mean(SE_X))
% RMSE_Y=sqrt(mean(SE_Y))
% RMSE_psi=sqrt(mean(SE_psi))
% 
% RMSE_SILAX=sqrt(mean(SE_SILA_X))
% RMSE_SILAY=sqrt(mean(SE_SILA_Y))
% RMSE_psicomp=sqrt(mean(SE_psicomp))

close all;
figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,1,1)
hold on;
grid on;
for i=496:726
    if X_sig(i)==1
        h(3)=plot(tempo(i)-9.1, LSx(i),'r*', 'MarkerSize',markersize);%plot SILA
    end
    h(2)=plot([tempo(i) tempo(i+1)]-9.1, [Px_est(i) Px_est(i+1)], 'b', 'LineWidth', linewidth);%plot Kalman
end 
h(1)=plot([0 21], [0.50 1.97], 'k','LineWidth', linewidth); %plot real
axis([0 20 0.5 2.1])
ylabel('Position X [m]') %,'FontSize',fontsize)
lgd=legend([h(1) h(2) h(3)],'Real position','Navigation System estimated position [10 Hz]','ALOS estimated position [1 Hz]');
lgd.Location='southeast';
hold off;
ax = gca;
ax.FontSize = fontsize; 


subplot(2,1,2)
hold on;
grid on;
for i=496:726
    if X_sig(i)==1
        h(3)=plot(tempo(i)-9.1, LSy(i),'r*','MarkerSize',markersize);%plot SILA
    end
    h(2)=plot([tempo(i) tempo(i+1)]-9.1, [Py_est(i) Py_est(i+1)], 'b','LineWidth', linewidth); %plot Kalman
end 
h(1)=plot([0 21], [0.70 2.08], 'k','LineWidth', linewidth); %plot real
axis([0 20 0.5 2.1])
ylabel('Position Y [m]') %,'FontSize',fontsize)
xlabel('Time [s]') %,'FontSize',fontsize)
lgd=legend([h(1) h(2) h(3)],'Real position','Navigation System estimated position [10 Hz]','ALOS estimated position [1 Hz]');
lgd.Location='southeast';
hold off;
ax = gca;
ax.FontSize = fontsize;

pause(1);
output_img = 'straight_SILA_Kalman_real';
print(output_img,'-dpng');
% command = ['mv', ' ' , output_img, '.png ./figures/'];
% system(command);