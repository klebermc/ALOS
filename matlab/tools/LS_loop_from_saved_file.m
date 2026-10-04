close all
n_emitters=8;
module_position_lms=[];
module_position_taylor=[];
module_position_aml=[];
for j=1:37
            aux_meas_t_h = [0 0 0 0 0 0 0];
            try
                available_modules = [];
                available_measurements = [];
                MODULE_246_OK=0;
                
                    for i=1:min(size(measured_time_history))
                        meas_index =  i; % The ultrasonic emiter module number
                        meas_time =  measured_time_history(j,i); % the time measured
                        meas_dist_aprox = (meas_time * 1e-6) * sound_speed; %the measured distance between the pair US emitter-receiver

                        if meas_index>=1 && meas_index<=n_emitters
                            if  meas_time>100 && meas_time<15000
                                %only consider as available, modules that the
                                %measurement appears in the message, and that are
                                %less then 15ms (max time waited to be out of range)
                                available_modules = [available_modules; stationary_modules_positions(meas_index,:)];
                                available_measurements = [available_measurements, meas_dist_aprox];

                                %Displays the value at the interface
%                                 set(handles.(sprintf('d%d_value', meas_index)),'string', num2str(meas_dist_aprox,4));
%                                 set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[0.3 0.6 0.3]);
                                if meas_index==2 || meas_index==4 || meas_index==6 || meas_index==8
                                    MODULE_246_OK = 1;
                                end
                            end
                            if meas_time<100
                                %makes the interface red, showing the valeus are
                                %out of range (restart receiver)
%                                 set(handles.(sprintf('d%d_value', meas_index)),'string', parsemsg{i+1});
%                                 set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[1 0.4 0.4]);
                            end
                            aux_meas_t_h(meas_index) = meas_time;
                        end
                    end

%                     measured_time_history=[measured_time_history; aux_meas_t_h];

                %fprintf('\n');
                %disp('***********************');
                %disp(strcat('AVAILABLE MEASUREMENTS: [', num2str(max(size(available_measurements))),']'));
                %disp('***********************');
                %fprintf('\n');
%                 availability_history=[availability_history, max(size(available_measurements))];
                
%                 set(handles.available_meas,'string',num2str(max(size(available_measurements)))); 
                % Enters this if when:
                % -calibration already done -> sound_speed diff 0
                % -there are more then 4 distance mesurements (needed for LMS)
                % -One of the measurements is from module 2, 4 or 6 (they have different heights, so it makes the matrix inversible at LMS)
                if(sound_speed~=0) && (max(size(available_measurements))>=4) && (MODULE_246_OK==1)
                        
                    %After the sensors measurements, calculate the location
                    [module_x,module_y,module_z] = LMS(available_modules, available_measurements);
                    %if (module_x^2+module_y^2+module_z^2)> 100
                    %    [mx_ts,my_ts,mz_ts] = taylor_series(available_modules,available_measurements,[0,0,0]);
                    %    [mx_aml,my_aml,mz_aml] = aml(available_modules,available_measurements,[0,0,0]);
                    %else
                        [mx_ts,my_ts,mz_ts] = taylor_series(available_modules,available_measurements,[module_x,module_y,module_z]);
                        
%                         if j==25
%                         [mx_aml,my_aml,mz_aml] = aml(available_modules,available_measurements,[0,0,0]);
%                         pause
%                         else
                        [mx_aml,my_aml,mz_aml] = aml(available_modules,available_measurements,[module_x,module_y,module_z]);
%                         end
                                            
                    %end
                    
                    module_position_lms = [module_position_lms; module_x,module_y,module_z];
                    module_position_taylor = [module_position_taylor; mx_ts,my_ts,mz_ts];
                    module_position_aml = [module_position_aml; mx_aml,my_aml,mz_aml];
                    
                    %For user information, some information is printed to
                    %the command line and to the interface
                    %fprintf('\n');
                    %disp('********************');
                    %disp(strcat('LOGGED VALID POSITIONS: [', num2str(size(module_position_lms,1)), ']'));
                    %disp(strcat('MODULE POSITION: [',num2str(module_x),' , ',num2str(module_y),' , ',num2str(module_z),']'));
                    %disp('********************');
                    %fprintf('\n');
                    subplot(3,1,1); 
                        grid on; hold on; 
                        plot(j, module_x,'bx');
                        plot(j, mx_ts,'rx');
                        plot(j, mx_aml,'kx');
                        hold off;
                    subplot(3,1,2); 
                        grid on; hold on; 
                        plot(j, module_y,'bx');
                        plot(j, my_ts,'rx');
                        plot(j, my_aml,'kx');
                        hold off;
                    subplot(3,1,3); 
                        grid on; hold on; 
                        plot(j, module_z,'bx');
                        plot(j, mz_ts,'rx');
                        plot(j, mz_aml,'kx');
                        hold off;
                        
                    if mx_ts>0 && mx_ts<3 && my_ts>0 && my_ts<3 && mz_ts>0 && mz_ts<3
                        %If there is a good position calculated, do not warn the quadrotor
                        MEASUREMENTS_NOK=0;
                    end
            
                end
            catch exception
                %disp(exception);
            end
end