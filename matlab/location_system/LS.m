function [x, y, z] = LS( fixed_modules_positions , distance_readings )

    num_fixed_modules = size(fixed_modules_positions,1);
    % ---------------------------------------------------------------------- %
    
    %TOA - LS
    % Take the values from the inputs
    for i=1:num_fixed_modules
        x(i)=fixed_modules_positions(i,1);
        y(i)=fixed_modules_positions(i,2);
        z(i)=fixed_modules_positions(i,3);
    
        k(i)=x(i)^2+y(i)^2+z(i)^2;
    
        m(i) = distance_readings(i);
    end
    
    
    % Build the matrixes A and b, for the system
	% 2At=b; - system
	% Quanto tenho emissores em posições diferentes no eixo z (pelo menos um
	% deles é diferente)
	
    % A=[(x2-x1) (y2-y1) (z2-z1);
    %    (x3-x1) (y3-y1) (z3-z1);
    %    (x4-x1) (y4-y1) (z4-z1)];
    
	% Se considerar que todos os emissores estão na mesma altura, a solução
	% vira apenas isso, mas assim não tenho informação de z, só de x,y
	% z tem que ser obtido de outra forma (ref eckert)
	% A=[(x2-x1) (y2-y1);	(x3-x1) (y3-y1);	(x4-x1) (y4-y1)];

	% b= [(m1^2 - m2^2 + k2 - k1);
	%	  (m1^2 - m3^2 + k3 - k1);
	% 	  (m1^2 - m4^2 + k4 - k1)];
    
    % t = (1/2) * inv(A'*A) * A'*b -> solucao
    
    A=[];
    b=[];
    for i=2:num_fixed_modules
        A = [A; (x(i)-x(1)) (y(i)-y(1)) (z(i)-z(1))];
        b = [b; (m(1)^2 - m(i)^2 + k(i) - k(1))];
    end
	
	t = (1/2) * inv(A'*A) * A'* b;
	% ---------------------------------------------------------------------- %
    
    %Assign output
    x=t(1);    y=t(2);    z=t(3);
    %keyboard;
end

