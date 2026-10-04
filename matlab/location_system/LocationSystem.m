clc; clear; close all; close all hidden;

[connect, send, receive, disconnect]=Serial_Comm;

% ----- Declaration of Variables ---------

%global variables
global END_EXECUTION;
global CONNECT_FS;
global CONNECT_robot;
global AUTOMATIC_MEASUREMENTS;
global CALIB_SOUND_SPEED;

global fixed_station_dev;
global robot_comm_dev;
global dt;
global sound_speed;
global manual_single_read;
global working_receiver;

%flags
END_EXECUTION = 0;
CONNECT_FS=0;
AUTOMATIC_MEASUREMENTS=0;
CALIB_SOUND_SPEED=0;
MEASUREMENTS_NOK=0; %the number of measurements that should be done, but went wrong

fixed_station_dev=0;
robot_comm_dev=0;
manual_single_read = [];
% sound_speed = 0;
sound_speed = 335.75;

working_receiver=2; % the receiver I am using at the moment.

%time measurement variables
tLoop=0;
tMessageExchange=0;
dt=1;

message='';

%System parameters
global n_receivers,
global n_emitters;
global stationary_modules_positions;

n_receivers=1;

stationary_modules_positions = [
                                0.000  0.05  2.48;
                                1.285  0.05  2.96;
                                2.535  0.05  2.48; 
                                2.915  1.60  1.99; 
                                2.570  3.05  2.48; 
                                1.260  3.05  2.96;
                                -0.05  2.90  2.48;
                                -0.47  1.52  1.77
                                ];
n_emitters = size(stationary_modules_positions,1);

WAYPOINTS = [   
2.10 2.40 0; %p9
1.25 2.40 0; %p6
0.50 2.40 0; %p3
1.25 1.50 0; %p5
0.50 0.70 0; %p1
1.25 0.70 0; %p4
2.10 0.70 0; %p7
1.25 1.50 0; %p5
            ];
        
module_x = 0; module_y = 0; module_z = 0;
mx_ts=0;      my_ts=0;      mz_ts=0;
mx_aml=0;     my_aml=0;     mz_aml=0; 
module_position_ls=[];
module_position_taylor = [];
module_position_aml = [];
availability_history=[];
measured_time_history=[];
robot_calc_position=[];
iter_wp=1;
positionX=0; positionY=0; positionZ=0;

dtKalman = 0.1; %the time step that Kalman filter runs the propagation, and also the time position controller runs
% numDF=0.3935;denDF=[1, -0.6065];
numDF=0.6321;denDF=[1, -0.3679];

contador=0;
%get the handle of the GUI
hGui = interface;
if ~isempty(hGui)
	% get the handles to the controls of the GUI
    handles = guidata(hGui);
else
    handles = [];
end

%Calls the script to initialize the values at the User Interface
init_interface_values
pause(0.1); %give time to the interface to be updated

tLoopStart=tic;

while END_EXECUTION ~= 1
    %clc;
    fprintf('\n\n---------------------------------------------------------------------------------------------\n\n');
	disp('--------- loop ----------');
	disp(strcat('Processing time [ms]: ',num2str(round(tLoop*1000))));
	disp(strcat('Loop Frequency [Hz]: ',num2str(1/dt)));
	disp('--------- ---- ----------');
    fprintf('\n');
    message='';    
    
    if CONNECT_FS==1        
        if CALIB_SOUND_SPEED==1
           %Calls the script for the calibration procedure
           ss_calib_procedure
        end
        
        if AUTOMATIC_MEASUREMENTS == 0
            %Doing manual measurements 
            if size(manual_single_read,2) > 0
                for i=1:max(size(manual_single_read))
                    % Request the emission a ultrasonic signal
                    send(fixed_station_dev, strcat('U',num2str(manual_single_read(i)),'!'));
                    disp(receive(fixed_station_dev));
                    
                    % Request the read value
                    send(fixed_station_dev, strcat('D',num2str(working_receiver),'!'));
                    message = receive(fixed_station_dev);
                    disp(message);

                    try
                        parsemsg = strsplit(message,{'-','.','\n'});
                        if str2double(parsemsg{2})<100
                            %Reinicialize the module
                            msgbox({'Receiver module is presenting wrong measurements,';'Turn the receiver module OFF and ON again!'}, 'Receiver Module Error', 'error')
                        end
                        meas_time = str2double(parsemsg{2}) * 1e-6;
                        meas_dist_aprox = meas_time * sound_speed;

                        disp(strcat('Emitter=', parsemsg{1}, ' Time=', num2str(meas_time)));
                        set(handles.(sprintf('d%d_value', manual_single_read(i))), 'string', num2str(meas_dist_aprox));
                    catch
                        disp('Invalid message received!')
                    end
                    pause(0.05);
                end
                manual_single_read=[]; %This garantees I'll do only one measurement
            end
        else
            MEASUREMENTS_NOK=MEASUREMENTS_NOK+1;
                    
            %Doing automatic measurements 
            %Send the command to start the distance measurement through
            %ultrasonic signal
            tStart = tic;  % TIC, pair 2 
                message = ultrasonic_measurements( n_emitters, handles, fixed_station_dev, 'all' );
            tMessageExchange = toc(tStart);  % TOC, pair 2 
            
            fprintf('\n');            
            disp('--------- msgs ----------');
            fprintf('*************\n%s\n*************\n',message);
            disp(strcat('Time for measuring dists (ms): ',num2str(round(tMessageExchange*1000))));
            disp('--------- ---- ----------');
            fprintf('\n');

            aux_meas_t_h = [0 0 0 0 0 0 0];
            try
                parsemsg = strsplit(message,{'-','.','\n'});
                available_modules = [];
                available_measurements = [];
                MODULE_246_OK=0;
                
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
                                set(handles.(sprintf('d%d_value', meas_index)),'string', num2str(meas_dist_aprox,4));
                                set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[0.3 0.6 0.3]);
                                if meas_index==2 || meas_index==4 || meas_index==6 || meas_index==8
                                    MODULE_246_OK = 1;
                                end
                            end
                            if meas_time<100
                                %makes the interface red, showing the valeus are
                                %out of range (restart receiver)
                                set(handles.(sprintf('d%d_value', meas_index)),'string', parsemsg{i+1});
                                set(handles.(sprintf('d%d_value', meas_index)),'BackgroundColor',[1 0.4 0.4]);
                            end
                            aux_meas_t_h(meas_index) = meas_time;
                        end
                    end

                    measured_time_history=[measured_time_history; aux_meas_t_h];
                else
                    disp(strcat('MSG ERROR [',tmp,'] [',num2str(max(size(parsemsg_tmp))),']'));
                    set(handles.err_nrf, 'Visible', 'On'); %error messages
                end

                %fprintf('\n');
                %disp('***********************');
                %disp(strcat('AVAILABLE MEASUREMENTS: [', num2str(max(size(available_measurements))),']'));
                %disp('***********************');
                %fprintf('\n');
                availability_history=[availability_history, max(size(available_measurements))];
                
                set(handles.available_meas,'string',num2str(max(size(available_measurements)))); 
                
                % Enters this if when:
                % -calibration already done -> sound_speed diff 0
                % -there are more then 4 distance mesurements (needed for LS)
                % -One of the measurements is from module 2, 4 or 6 (they have different heights, so it makes the matrix inversible at LS)
                if(sound_speed~=0) && (max(size(available_measurements))>=4) && (MODULE_246_OK==1)
                    %After the sensors measurements, calculate the location
                    [module_x,module_y,module_z] = LS(available_modules, available_measurements);
                    [mx_ts,my_ts,mz_ts] = taylor_series(available_modules,available_measurements,[module_x,module_y,module_z]);
                    [mx_aml,my_aml,mz_aml] = aml(available_modules,available_measurements,[1.5,1.5,0]);
                    
                    module_position_ls = [module_position_ls; module_x,module_y,module_z];
                    module_position_taylor = [module_position_taylor; mx_ts,my_ts,mz_ts];
                    module_position_aml = [module_position_aml; mx_aml,my_aml,mz_aml];
                    
                    %For user information, some information is printed to
                    %the command line and to the interface
                    %fprintf('\n');
                    %disp('********************');
                    %disp(strcat('LOGGED VALID POSITIONS: [', num2str(size(module_position_ls,1)), ']'));
                    %disp(strcat('MODULE POSITION: [',num2str(module_x),' , ',num2str(module_y),' , ',num2str(module_z),']'));
                    %disp('********************');
                    %fprintf('\n');
                    
                    
                    set(handles.mobile_module_position_ls, 'String',strcat('(',num2str(module_x,3),' , ', num2str(module_y,3),' , ', num2str(module_z,3),')'));
                    set(handles.mobile_module_position_ts, 'String',strcat('(',num2str(mx_ts,3),' , ', num2str(my_ts,3),' , ', num2str(mz_ts,3),')'));
                    
                    set(handles.logged_meas, 'string',num2str(size(module_position_ls,1))); 
                    
                    %Plot the positions at the interface
%                     delete(mod_plot_position);
%                     delete(mod_plot_line);
%                     mod_plot_position = plot3(handles.plot_pos, max(module_x,0),max(module_y,0),max(module_z,0),'bo');
%                     mod_plot_line = plot3(handles.plot_pos, [module_x module_x],[module_y module_y],[0 module_z],'b','LineWidth',1.5);
                    
%                     delete(mod_plot_position_TS);
%                     delete(mod_plot_line_TS);
%                     mod_plot_position_TS = plot3(handles.plot_pos, max(mx_ts,0),max(my_ts,0),max(mz_ts,0),'rx');
%                     mod_plot_line_TS = plot3(handles.plot_pos, [mx_ts mx_ts],[my_ts my_ts],[0 mz_ts],'r','LineWidth',1.5);
                    
                    %delete(mod_plot_position_AML);
                    %delete(mod_plot_line_AML);
                    %mod_plot_position_AML = plot3(handles.plot_pos, max(mx_aml,0),max(my_aml,0),max(mz_aml,0),'k^');
                    %mod_plot_line_AML = plot3(handles.plot_pos, [mx_aml mx_aml],[my_aml my_aml],[0 mz_aml],'k','LineWidth',1.5);
                    
                    if mx_ts>0 && mx_ts<3 && my_ts>0 && my_ts<3 && mz_ts>0 && mz_ts<3
                        %If the calculated position is inside the test area, do not send any stop message to the robot
                        MEASUREMENTS_NOK=0;
                    end
                    
                    % ************* Spikes removal*******************
                    % If the last calculated position is more then X meters distant from the last one, send the median,
                    % not the calculated value
                    last_pos=size(module_position_taylor,1);
                    if last_pos>=5
                        %previous considered point
                        x1=median(module_position_taylor(last_pos-3:last_pos-1,1)); 
                        y1=median(module_position_taylor(last_pos-3:last_pos-1,2)); 
                        z1=median(module_position_taylor(last_pos-3:last_pos-1,3));
                        
                        %actual point
                        x2=module_position_taylor(last_pos,1);   
                        y2=module_position_taylor(last_pos,2);   
                        z2=module_position_taylor(last_pos,3);
                        
                        if sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2)>0.6 % I just did a wrong measurement (spike)
                            mx_ts = median(module_position_taylor(last_pos-3:last_pos-1,1));
                            my_ts = median(module_position_taylor(last_pos-3:last_pos-1,2)); 
                            mz_ts = median(module_position_taylor(last_pos-3:last_pos-1,3));
                        end
                    end
                    robot_calc_position = [robot_calc_position; mx_ts my_ts mz_ts];            
                    
                end
            catch exception
                %disp(exception);
            end
        end
    end
    
    if CONNECT_robot==1
        %Send the position of the module to the robot arduino embeddeed
        %the position is in [mm], because this way there is no need to
        %transmit a value with decimal numbers, only an integer
            
        last_pos=size(robot_calc_position,1);
        if last_pos>11
            if MEASUREMENTS_NOK >= 3
                %There were more than 3 seconds without a valid position calculated, make the robot stop
                positionX=-1;
                positionY=-1;
                positionZ=-1;
            else
                %do not send values out of the test area to the robot  
                %With low pass filter
%                 filteredVectX = filter(numDF,denDF,robot_calc_position(last_pos-10:last_pos,1));
%                 filteredVectY = filter(numDF,denDF,robot_calc_position(last_pos-10:last_pos,2));
%                 filteredVectZ = filter(numDF,denDF,robot_calc_position(last_pos-10:last_pos,3));
%                 positionX = max(min(filteredVectX(end),3),0);
%                 positionY = max(min(filteredVectY(end),3),0);
%                 positionZ = max(min(filteredVectZ(end),3),0);

                %Without low pass filter
                positionX = max(min(robot_calc_position(last_pos,1),3),0);
                positionY = max(min(robot_calc_position(last_pos,2),3),0);
                positionZ = max(min(robot_calc_position(last_pos,3),3),0);

                delete(mod_plot_position_AML);
                delete(mod_plot_line_AML);
                mod_plot_position_AML = plot3(handles.plot_pos, positionX, positionY, positionZ,'k^');
                mod_plot_line_AML = plot3(handles.plot_pos, [positionX positionX],[positionY positionY],[0 positionZ],'k','LineWidth',1.5);
                
            end

%             positionX=1.25; positionY=1.50;positionZ=0; 
            
            %positionX=0; positionY=0;positionZ=0; 
            %Current robot position
            if iter_wp>size(WAYPOINTS,1) 
                positionX=-1;
                positionY=-1;
                positionZ=-1;
            end
            msg2robot=strcat(num2str(int16(positionX*1000)),',',num2str(int16(positionY*1000)),',',num2str(int16(positionZ*1000)),',');

            %Update the current waypoint if needed
            [iter_wp,current_wp] = waypoints_iterator(iter_wp,[positionX positionY positionZ], WAYPOINTS);
%             current_wp = WAYPOINTS(1,:);
%             contador=contador+1;
%             if contador < 10
%                 current_wp=[0 0 0];
%             end
%             if contador>=10 && contador<30
%                 current_wp=[0 0 1];
%             end
%             if contador > 30
%                 current_wp=[0 0 0];
%             end
            msg2robot=strcat(msg2robot,num2str(int16(current_wp(1)*1000)),',',num2str(int16(current_wp(2)*1000)),',',num2str(int16(current_wp(3)*1000)),',\n');

            %Updates the current waypoint in value in the UI, and plot the
            %waypoint in the 3d graphic
            set(handles.current_wp, 'String',strcat('(',num2str(current_wp(1),3),' , ',num2str(current_wp(2),3),' , ',num2str(current_wp(3),3),')'));
            delete(wp_position);
            wp_position = plot3(handles.plot_pos, current_wp(1),current_wp(2),current_wp(3),'bp');
            
            fprintf('\n');
            disp('***********************');
            disp(msg2robot);
            disp('***********************');
            fprintf('\n');
            try
                %Send the information to robot
                send(robot_comm_dev, msg2robot);
            catch
                END_EXECUTION=1;
            end
        end
    end
    
    tLoop=toc(tLoopStart);
    if  tLoop<dt
		pause(dt-tLoop);
    else
        %A little time for other threads inside the processor
        pause(0.05);
    end
    tLoopStart=tic; 
end

toc(tLoopStart);
try
    close(hGui);
end
if fixed_station_dev~=0 && CONNECT_FS~=0; 
    disconnect(fixed_station_dev); 
end
if robot_comm_dev~=0 && CONNECT_robot~=0;
    disconnect(robot_comm_dev); 
end
disp('FIM DE EXECUCAO');
