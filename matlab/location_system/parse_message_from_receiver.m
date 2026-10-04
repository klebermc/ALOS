function [ available_measurements, available_modules, MODULE_2468_OK, aux_meas_t_h ] = parse_message_from_receiver( working_receiver_pos, message, handles)
%function [ available_measurements, available_modules, MODULE_2468_OK, measured_time_history ] = parse_message_from_receiver( message, handles, sound_speed, measured_time_history, n_emitters )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global sound_speed;
global n_emitters;
global stationary_modules_positions;


    parsemsg = strsplit(message,{'-','.','\n'});
    available_modules = [];
    available_measurements = [];
    MODULE_2468_OK=0;

    %parsemsg
    if max(size(parsemsg))>10
        for i=1:2:(max(size(parsemsg)) - mod(max(size(parsemsg)),2))
            meas_index = str2double(parsemsg{i}); % The ultrasonic emiter module number
            meas_time =  str2double(parsemsg{i+1}); % the time measured
            meas_dist_aprox = (meas_time * 1e-6) * sound_speed; %the measured distance between the pair US emitter-receiver

            if meas_index>=1 && meas_index<=n_emitters
                if  meas_time>100 && meas_time<15000
                    %only consider as available, modules that the
                    %measurement appears in the message, and that are
                    %less then 15ms (max time waited to be out of range)
                    available_modules = [available_modules; stationary_modules_positions(meas_index,:)];
                    available_measurements = [available_measurements, meas_dist_aprox];

                    %Displays the value at the interface
                    %only shows the values of the first receiver declared at the vector at the interface
                    if(working_receiver_pos==1)
                        set(handles.(sprintf('d%d_value', meas_index)),'string', num2str(meas_dist_aprox,4));
                        set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[0.3 0.6 0.3]); 
                    end
                    if meas_index==2 || meas_index==4 || meas_index==6 || meas_index==8
                        MODULE_2468_OK = 1;
                    end
                end
                if meas_time<100
                    %makes the interface red, showing the valeus are
                    %out of range (restart receiver)
                    if(working_receiver_pos==1) 
                        set(handles.(sprintf('d%d_value', meas_index)),'string', parsemsg{i+1});
                    	set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[1 0.4 0.4]);
                    end
                end
                aux_meas_t_h(meas_index) = meas_time;
            end
        end

        %measured_time_history=[measured_time_history; aux_meas_t_h];
    else
        disp(strcat('MSG ERROR [',tmp,'] [',num2str(max(size(parsemsg_tmp))),']'));
        set(handles.err_nrf, 'Visible', 'On'); %error messages
    end
end

