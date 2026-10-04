close all;
clear A aux11 aux12 aux13 aux21 aux22 aux23 aux31 aux32 aux33 t B aux1 aux2 aux3
clear g h f r k

%first guess
mx=0;my=0;mz=0;
%m_x=1.5;m_y=1.5;m_z=0;
%mx = 1.1745; my = 0.3574; mz = 0.0842;
Q = eye(7)*0.1; 

for i=1:7
    x(i)=stationary_modules_positions(i,1);
    y(i)=stationary_modules_positions(i,2);
    z(i)=stationary_modules_positions(i,3);

    k(i)=x(i)^2+y(i)^2+z(i)^2;

    %m(i) = distance_readings(i);
end  
    
Jn_1=0;

hold on;
grid on;
    
for j=1:100
    s = mx^2 + my^2 + mz^2;

    for i=1:7
        %não manjei se faço isso sempre ou só pro initial guess
        r(i,1) = sqrt((mx - x(i))^2 + (my - y(i))^2 + (mz - z(i))^2);

        %[(sqrt((m_x - x(i))^2 + (m_y - y(i))^2 + (m_z - z(i))^2)) , (s + k(i) -2*(m_x*x(i) + m_y*y(i) + m_z*z(i)))]
        %r(i,1) = s + k(i) -2*(m_x*x(i) + m_y*y(i) + m_z*z(i));
        g(i) = (mx - x(i)) / (r(i)*(r(i)+m(i)));
        h(i) = (my - y(i)) / (r(i)*(r(i)+m(i)));
        f(i) = (mz - z(i)) / (r(i)*(r(i)+m(i)));
    end

    %funcao custo
    J = (m - r)' * inv(Q) * (m - r)
    
    %condicao de parada
    %     [j abs(J-Jn_1)/Jn_1]
%     if abs(J-Jn_1)/Jn_1 <= (3/100)
%         break;
%     end
%     Jn_1=J;
    
    %Computing the A matrix
    for i=1:7
        aux11(i) = g(i)*x(i); aux12(i) = g(i)*y(i); aux13(i) = g(i)*z(i);
        aux21(i) = h(i)*x(i); aux22(i) = h(i)*y(i); aux23(i) = h(i)*z(i);
        aux31(i) = f(i)*x(i); aux32(i) = f(i)*y(i); aux33(i) = f(i)*z(i);
    end
    A = [ sum(aux11), sum(aux12), sum(aux13);
          sum(aux21), sum(aux22), sum(aux23);
          sum(aux31), sum(aux32), sum(aux33) ];
        
    %Computing the B matrix
    for i=1:7
        aux1(i) = g(i)*(s+k(i)-m(i)^2);
        aux2(i) = h(i)*(s+k(i)-m(i)^2);
        aux3(i) = f(i)*(s+k(i)-m(i)^2);
    end
    B = [ sum(aux1); sum(aux2); sum(aux3) ];
   
    t = (1/2) * inv(A'*A) * A'* B;
    
    plot(j,mx,'rx');
    plot([j j+1],[mx t(1)],'r');
    plot(j,my,'bo');
    plot([j j+1],[my t(2)],'b');
    plot(j,mz,'k*');
    plot([j j+1],[mz t(3)],'k');
    
    mx=t(1);
    my=t(2);
    mz=t(3);
    
    pause(0.5);
   % pause
end

t