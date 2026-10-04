function [x, y, z] = taylor_series( fixed_modules_positions , distance_readings, firt_guess )

    %first guess
    %mx=1.5;my=1.5;mz=0;
    mx = firt_guess(1); my = firt_guess(2); mz = firt_guess(3);
    num_meas = max(size(distance_readings));
    Q = eye(num_meas)*0.1; 

    for i=1:num_meas
        x(i)=fixed_modules_positions(i,1);
        y(i)=fixed_modules_positions(i,2);
        z(i)=fixed_modules_positions(i,3);

        m(i,1) = distance_readings(i);
    end  

    for j=1:100
        A=[];
        D=[];
        %Computing the D matrix
        
        for i=1:num_meas
            r(i,1) = sqrt((mx - x(i))^2 + (my - y(i))^2 + (mz - z(i))^2);
            
            %Computing the A matrix
            ai1 = (mx-x(i))/r(i);  ai2 = (my-y(i))/r(i);  ai3 = (mz-z(i))/r(i);
            A = [A; ai1, ai2, ai3];
            
            %Computing the D matrix
            %fi,v = fi(xv,yv,zv) = sqrt((xv-xi)^2+(yv-yi)^2+(zv-zi)^2)
            %D = [m1-f1,v , m2-f2,v , mn-fn,v];
            fiv = sqrt((mx-x(i))^2+(my-y(i))^2+(mz-z(i))^2);
            D = [D; (m(i)-fiv)];
        end

        delta = inv( A'*inv(Q)*A ) * A'*inv(Q)*D ;

        previous = [mx; my; mz];

        mx=mx + delta(1);
        my=my + delta(2);
        mz=mz + delta(3);
        
        %distancia entre os pontos
        J = sqrt((mx - previous(1))^2 + (my - previous(2))^2 + (mz - previous(3))^2);

        %condicao de parada
        if J <= 0.001 %1mm de diferença entre o ponto calculado na iteracao aterior e o atual
            break;
        end
    end
     %Assign output
     if (std(r-m)>0.05)
        tmp=-find(-abs(r-m) + max(abs(r-m))<0.001);
        z=tmp(1);
%         (r-m)'
%         disp([mean(r-m), std(r-m), -z])
     else
         z=mz;
     end
    x=mx;    y=my;%    z=mz;
end