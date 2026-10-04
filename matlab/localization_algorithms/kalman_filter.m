
stationary_modules_positions = [0.000  0.05  2.48;
                                1.285  0.05  2.96;
                                2.535  0.05  2.48; 
                                2.915  1.60  1.99; 
                                2.570  3.05  2.48; 
                                1.260  3.05  2.96;
                                -0.05  2.90  2.48];
num_stationary_modules = size(stationary_modules_positions,1);

%ponto escolhido para teste
x=1.5;
y=1.5;
z=0;

%Matrizes do sistema
P = eye(3);
H = eye(3);
Q = eye(3) * 0.01;
R = eye(3) * 0.2;

close all;
figure;
hold on;
grid on;
axis([0 5 0 2])
plot([0 5],[1.5 1.5])

%Calcula as distâncias entre o ponto escolhido e posição dos n módulos
for i=1:num_stationary_modules
    m(i) = sqrt( (x-stationary_modules_positions(i,1))^2 + (y-stationary_modules_positions(i,2))^2 + (z-stationary_modules_positions(i,3))^2 );
    m(i) = m(i) + 0.15*randn;
end


% Predict
%xhat = a * xhat;
%p    = a * p * a;
[module_x,module_y,module_z] = trilateration(stationary_modules_positions(1:4,:), m(1:4));
xhat = [module_x;module_y;module_z]
plot(0,xhat(1),'rx');

for i=1:4
    x(i)=stationary_modules_positions(i,1);
    y(i)=stationary_modules_positions(i,2);
    z(i)=stationary_modules_positions(i,3);
    k(i)=x(i)^2+y(i)^2+z(i)^2;
end

A=[];
B=[];
for i=2:4
    A = [A; (x(i)-x(1)) (y(i)-y(1)) (z(i)-z(1))];
    B = [B; (m(1)^2 - m(i)^2 + k(i) - k(1))];
end

dB_dX = [(x(2)-x(1)) (y(2)-y(1)) (z(2)-z(1)); (x(3)-x(1)) (y(3)-y(1)) (z(3)-z(1)); (x(4)-x(1)) (y(4)-y(1)) (z(4)-z(1))];
%dB_dX = [((x(2)+y(2)+z(2))-(x(1)+y(1)+z(1))); ((x(3)+y(3)+z(3))-(x(1)+y(1)+z(1))); ((x(4)+y(4)+z(4))-(x(1)+y(1)+z(1)))]
F_k   = inv( A' * A ) * A' * dB_dX;
P_k   = F_k * P * F_k' + Q

i=5;
while i<=num_stationary_modules
    % Update
    %g    = p  / (p  + r);
    %xhat = xhat + g * (z - xhat);
    %p    = (1 - g) * p;
    [meas1,meas2,meas3] = trilateration(stationary_modules_positions(1:i,:), m(1:i));
    meas=[meas1;meas2;meas3];
    G_k  = P_k*H' / (H*P_k*H' + R);
    xhat = xhat + G_k * (meas - xhat)
    P_k  = (eye(3) - G_k*H)*P_k
    plot((i-4),xhat(1),'rx');
    plot((i-4),meas1,'bo');
    i=i+1;
end
[mx,my,mz] = aml(stationary_modules_positions, m, [module_x,module_y,module_z]);
plot(5,mx,'kx');