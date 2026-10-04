global fixed_station_dev;
global n_emitters;
global stationary_modules_positions;
global sound_speed;
global CALIB_SOUND_SPEED;
global END_EXECUTION;
global working_receiver;

%Here is the calibration procedure
time_measurements=zeros(20,n_emitters);
    
%The position of the module for calibration
x_calib = 1.25;
y_calib = 1.50;
%z_calib = 0.33; %quad
z_calib = 0.06; %ground
%z_calib = 0.22; %ground robot

% x_calib = 1.5; y_calib = 1.5; z_calib = 0.0;
% message = '1-10854.2-10042.3-10206.4-10752.5-8145';

n_fails=0;
text41_box_value = get(handles.text41,'String'); %copy the text of the box into a local variable, this text is changed inside this script
set(handles.sound_speed_value,'BackgroundColor',[0.8 0.8 0.8]); %make the background of the sound speed display gray

% Capturing 10 time measurements from all sensors

for emitter=1:n_emitters
    set(handles.(sprintf('d%d_value', emitter)),'BackgroundColor',[0.8 0.8 0.8]);
    set(handles.(sprintf('d%d_value', emitter)),'string', '');
end

emitter=1;
while emitter<=n_emitters
    if END_EXECUTION == 1
        break;
    end
    num_calib_data=1;
    
    while num_calib_data<=20 &&  END_EXECUTION~=1
        n_fails=n_fails+1; % consider it has failed already, if it succed, turn this number into 0
        if(n_fails>20)
            break;
        end
            
        message='';
        send(fixed_station_dev, strcat('U',num2str(emitter),'!'));
        disp(receive(fixed_station_dev));
        pause(0.1);
        %          message = receive(fixed_station_dev);
        %disp(message);
        
        % 2.4GHz RF signal
        send(fixed_station_dev, strcat('D',num2str(working_receiver(1)),'!'));
        message = receive(fixed_station_dev);
        disp(message);
        
        try
            parsemsg = strsplit(message,{'-','.','\n'});
            %if max(size(parsemsg))>=size(stationary_modules_positions,1)*2
            if max(size(parsemsg))>=2
                %If there are measurements from all modules, get in here
                for i=1:2:(max(size(parsemsg)) - mod(max(size(parsemsg)),2))
                    time_index = str2num(parsemsg{i});
                    time_measurements(num_calib_data,time_index) = (str2double(parsemsg{i+1}) * 1e-6);
                end
                              
                set(handles.text41,'String', 'Calib Data Collected:');
                set(handles.sound_speed_value,'String',num2str(num_calib_data))
                if time_measurements(num_calib_data,time_index)<0.015 && time_measurements(num_calib_data,time_index)>0.0001
                    fprintf('OK - %d\n', num_calib_data);
                    set(handles.(sprintf('d%d_value', time_index)),'string', num2str(time_measurements(num_calib_data,time_index),4));
                    set(handles.(sprintf('d%d_value', time_index)),'BackgroundColor',[0.3 0.6 0.3]);
                    n_fails=0;
                    num_calib_data = num_calib_data +1;
                else
                    fprintf('N-OK - %.3f\n', time_measurements(num_calib_data,time_index));
                    set(handles.(sprintf('d%d_value', time_index)),'string', num2str(time_measurements(num_calib_data,time_index),4));
                    set(handles.(sprintf('d%d_value', time_index)),'BackgroundColor',[1 0.4 0.4]);
                end
            end
        catch exception
            disp(exception);
            disp('Something went wrong!');
        end
        pause(0.1);
    end
    emitter=emitter+1;
end
set(handles.text41,'String', text41_box_value);

if(n_fails<20)
    %Iterate, testing different values for the the sound speed until I found the one with smaller error.
    soundspeed = 200;
    best_values=[100 soundspeed];
    distance_measurements=[];
    %error=[];
    while soundspeed<400
        if END_EXECUTION == 1
            break;
        end
        for i=1:size(time_measurements,2)
            distance_measurements(i) = median(time_measurements(:,i)) * soundspeed;
        end

        [module_x,module_y,module_z] = LS(stationary_modules_positions, distance_measurements);
        [module_x,module_y,module_z] = taylor_series(stationary_modules_positions, distance_measurements, [module_x,module_y,module_z] );
        dist_from_real_point = sqrt( (x_calib-module_x)^2 + (y_calib-module_y)^2 + (z_calib-module_z)^2 );
        if dist_from_real_point < best_values(1)
            best_values(1)=dist_from_real_point;
            best_values(2)=soundspeed;
        end

        %error=[error; sqrt( (x_calib-module_x)^2 + (y_calib-module_y)^2 ), dist_from_real_point];
        if mod(int32(soundspeed*100),500)==0
            %Show the value of the best sound speed calculated a few times during this loop
            %show this value every iteration slows down the simulation
            disp(strcat('Error=',num2str(dist_from_real_point,4),'m, Sound Speed=',num2str(soundspeed,5),'m/s'));
            set(handles.sound_speed_value,'String',num2str(best_values(2)));
            pause(0.01);
        end
        soundspeed=soundspeed+0.01;
    end

    pause(0.1);
    disp(strcat('End of calibration, actual sound speed [', num2str(best_values(2),5),' m/s]'));
    sound_speed = best_values(2);
    
    % Displays the sound speed
    set(handles.sound_speed_value,'String',num2str(sound_speed))
    set(handles.text41,'String', text41_box_value);
    set(handles.sound_speed_value,'BackgroundColor',[0.3 0.75 0.3]);
    
    % Only when there is a valid value for the sound speed, activate the
    % buttons for measurements
    set(handles.auto_meas_tbutton, 'Visible', 'On'); %automatic measurements button
    set(handles.manual_read_pushbutton, 'Visible', 'On'); %manual measurements button
end
CALIB_SOUND_SPEED = 0;
    
% Clear the variables used only for calibration
clear best_values dist_from_real_point soundspeed time_measurements;
clear time_index num_calib_data text41_box_value;
clear x_calib y_calib z_calib;
