available_modules=[];
available_measurements=[];
module_position_taylor = [];

for i=1:max(size(m_t_h))
    MODULE_567_OK=0;
    available_modules=[];
    available_measurements=[];
    for j=1:7
        if (m_t_h(i,j))<14500 && (m_t_h(i,j))>100
            available_modules = [available_modules; stationary_modules_positions(j,:)];
            available_measurements = [available_measurements, m_t_h(i,j)*(1e-6)*sound_speed];

            if j>=5; MODULE_567_OK = 1; end
        end
    end

    if i==1127
        pause(0.1)
    end

    if(MODULE_567_OK==1)
        %After the sensors measurements, calculate the location
        [module_x,module_y,module_z] = trilateration(available_modules, available_measurements);
        [module_x,module_y,module_z] = taylor_series( available_modules , available_measurements, [module_x,module_y,module_z] );
        module_position_taylor = [module_position_taylor; module_x,module_y,module_z];
    end
    
%     [ a_h(i) max(size(available_measurements)) ]
end