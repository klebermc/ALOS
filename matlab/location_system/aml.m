function [x, y, z] = aml( fixed_modules_positions , distance_readings, firt_guess )

    %first guess
    mx = firt_guess(1); my = firt_guess(2); mz = firt_guess(3);
    num_meas = max(size(distance_readings));
    Q = eye(num_meas)*0.1; 

    for i=1:num_meas
        x(i)=fixed_modules_positions(i,1);
        y(i)=fixed_modules_positions(i,2);
        z(i)=fixed_modules_positions(i,3);

        k(i)=x(i)^2+y(i)^2+z(i)^2;

        m(i,1) = distance_readings(i);
    end  
    Jn_1=0;
    
    for j=1:100
        A=[];
        B=[];
        s = mx^2 + my^2 + mz^2;

        for i=1:num_meas
            r(i,1) = sqrt((mx - x(i))^2 + (my - y(i))^2 + (mz - z(i))^2);

            g(i) = (mx - x(i)) / (r(i)*(r(i)+m(i)));
            h(i) = (my - y(i)) / (r(i)*(r(i)+m(i)));
            f(i) = (mz - z(i)) / (r(i)*(r(i)+m(i)));
        end

        %funcao custo
        Jn = (m - r)' * inv(Q) * (m - r);
        if j==1
         Jn_1 = 100*Jn;
        end
        
        %condicao de parada
        if abs(Jn-Jn_1)/Jn_1 <= (3/100)
            break;
        end
        Jn_1=Jn;

        %Computing the A matrix
        for i=1:num_meas
            aux11(i) = g(i)*x(i); aux12(i) = g(i)*y(i); aux13(i) = g(i)*z(i);
            aux21(i) = h(i)*x(i); aux22(i) = h(i)*y(i); aux23(i) = h(i)*z(i);
            aux31(i) = f(i)*x(i); aux32(i) = f(i)*y(i); aux33(i) = f(i)*z(i);
        end
        A = [ sum(aux11), sum(aux12), sum(aux13);
              sum(aux21), sum(aux22), sum(aux23);
              sum(aux31), sum(aux32), sum(aux33) ];

        %Computing the B matrix
        for i=1:num_meas
            aux1(i) = g(i)*(s+k(i)-m(i)^2);
            aux2(i) = h(i)*(s+k(i)-m(i)^2);
            aux3(i) = f(i)*(s+k(i)-m(i)^2);
        end
        B = [ sum(aux1); sum(aux2); sum(aux3) ];

        t = (1/2) * inv(A)* B;

        mx=t(1);
        my=t(2);
        mz=t(3);
    end
     %Assign output
    x=mx;    y=my;    z=mz;
end