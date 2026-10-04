% Testando o filtro hampel
% 
% da página do matlab:
% 
% Filter Delay
% Note that the filtered output is delayed by about twelve hours. This is due to the fact that our moving average filter has a delay.
% 
% Any symmetric filter of length N will have a delay of (N-1)/2 samples. We can account for this delay manually.

mod_pos = module_position_taylor;

close all;
subplot(3,1,1); plot(mod_pos(:,1),'r'); hold on; grid on;
subplot(3,1,2); plot(mod_pos(:,2),'r'); hold on; grid on;
subplot(3,1,3); plot(mod_pos(:,3),'r'); hold on; grid on;

for i=5:max(size(mod_pos))
    mx = mod_pos(i,1);
    my = mod_pos(i,2);
    mz = mod_pos(i,3);
    
    if i>3
        hx = hampel(mod_pos(i-4:i,1),2,2);
        hy = hampel(mod_pos(i-4:i,2),2,2);
        hz = hampel(mod_pos(i-4:i,3),2,2);
                    
        pos=4;
        subplot(3,1,1); plot(i,hx(pos),'bx');
        subplot(3,1,2); plot(i,hy(pos),'bx');
        subplot(3,1,3); plot(i,hz(pos),'bx');
        pause(0.05);        
    end
%     subplot(3,1,1); plot(i,mx,'rx');
%     subplot(3,1,2); plot(i,my,'rx');
%     subplot(3,1,3); plot(i,mz,'rx');
%     
end



for i=5:max(size(mod_pos))
    mx = mod_pos(i,1);
    my = mod_pos(i,2);
    mz = mod_pos(i,3);
    
    if i>3

        dpx = std(mod_pos(i-4:i-1,1));
        p_mx = mod_pos(i-1,1);
        if mx > (p_mx-3*dpx) && mx < (p_mx+3*dpx)
            hx = mx;
        else
            %mod_pos(i,1) = p_mx;
            hx=p_mx;
        end
        
        dpy = std(mod_pos(i-4:i-1,2));
        p_my = mod_pos(i-1,2);
        if my > (p_my - 3*dpy) && my < (p_my+3*dpy)
            hy = my;
        else
            %mod_pos(i,2) = p_my;
            hy=p_my;
        end

        
        dpz = std(mod_pos(i-4:i-1,3));
        p_mz = mod_pos(i-1,3);
        if mz > (p_mz-3*dpz) && mz < (p_mz+3*dpz)
            hz = mz;
        else
            %mod_pos(i,3) = p_mz;
            hz=p_mz;
        end

        
        subplot(3,1,1); plot(i,hx,'kx');
        subplot(3,1,2); plot(i,hy,'kx');
        subplot(3,1,3); plot(i,hz,'kx');
        pause(0.05);        
    end
%     subplot(3,1,1); plot(i,mx,'rx');
%     subplot(3,1,2); plot(i,my,'rx');
%     subplot(3,1,3); plot(i,mz,'rx');
%     

end
