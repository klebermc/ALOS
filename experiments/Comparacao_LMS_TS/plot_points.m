load('ponto5_altura0.mat')

close all

figure
subplot(4,1,1)
plot(availability_history)
grid on;
subplot(4,1,2)
plot(module_position_taylor(:,1))
hold on;grid on;
plot(module_position(:,1))
legend('TS X','LMS X')
subplot(4,1,3)
plot(module_position_taylor(:,2))
hold on;grid on;
plot(module_position(:,2))
legend('TS Y','LMS Y')
subplot(4,1,4)
plot(module_position_taylor(:,3))
hold on;grid on;
plot(module_position(:,3))
legend('TS Z','LMS Z')

figure
subplot(2,3,1); plot(measured_time_history(:,1))
subplot(2,3,2);  plot(measured_time_history(:,2))
subplot(2,3,3);  plot(measured_time_history(:,3))
subplot(2,3,4); plot(measured_time_history(:,4))
subplot(2,3,5);  plot(measured_time_history(:,5))
subplot(2,3,6);  plot(measured_time_history(:,6))

[ std(module_position(:,1)) std(module_position(:,2)) std(module_position(:,3))]
[ std(module_position_taylor(:,1)) std(module_position_taylor(:,2)) std(module_position_taylor(:,3))]