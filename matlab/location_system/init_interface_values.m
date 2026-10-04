%System parameters
global n_emitters;
global stationary_modules_positions;

% Exclude from the UI the not used objects
for i = 1:10
    if i<= n_emitters
        set(handles.(sprintf('d%d_value', i)), 'Visible', 'On'); 
        set(handles.(sprintf('d%d_checkbox', i)), 'Visible', 'On'); 
    else
        set(handles.(sprintf('d%d_value', i)), 'Visible', 'Off'); 
        set(handles.(sprintf('d%d_checkbox', i)), 'Visible', 'Off'); 
    end
end

%Begins the figure plot
cla(handles.plot_pos)
plot3(handles.plot_pos, stationary_modules_positions(1,1),stationary_modules_positions(1,2),stationary_modules_positions(1,3),'rx'); %the point at the module position
hold(handles.plot_pos);
plot3(handles.plot_pos, [stationary_modules_positions(1,1) stationary_modules_positions(1,1)],[stationary_modules_positions(1,2) stationary_modules_positions(1,2)],[0 stationary_modules_positions(1,3)],'k','LineWidth',1.5); %the line to the ground
for i=2:n_emitters
    plot3(handles.plot_pos, stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3),'rx');%the point at the module position
    plot3(handles.plot_pos, [stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'k','LineWidth',1.5);%the line to the ground
end
mod_plot_position = plot3(handles.plot_pos, 0,0,0, 'bo'); %The to-be measured point
mod_plot_line = plot3(handles.plot_pos, [0 0],[0 0],[0 0],'b','LineWidth',1.5); %line to the ground

for i=1:length(working_receiver)
    mod_plot_position_TS(working_receiver(i)) = plot3(handles.plot_pos, 0,0,0,'rx');
    mod_plot_line_TS(working_receiver(i)) = plot3(handles.plot_pos, [0 0],[0 0],[0 0],'r','LineWidth',1.5); %using taylor series
    mod_plot_label_TS(working_receiver(i)) = text(handles.plot_pos, 0,0,0,'a');
end 

%mod_plot_position_AML = plot3(handles.plot_pos, 0,0,0,'k^');
%mod_plot_line_AML = plot3(handles.plot_pos, [0 0],[0 0],[0 0],'k','LineWidth',1.5); %using AML

wp_position = plot3(handles.plot_pos,0,0,0,'bp'); %the current waypoint 

set(handles.plot_pos,'XLim',[min(stationary_modules_positions(:,1)) max(stationary_modules_positions(:,1))],'YLim',[min(stationary_modules_positions(:,2)) max(stationary_modules_positions(:,2))],'ZLim',[0 max(stationary_modules_positions(:,3))]); %The axis limits
grid(handles.plot_pos);       %The grid lines
xlabel(handles.plot_pos,'X'); %The axis label
ylabel(handles.plot_pos,'Y'); %The axis label
zlabel(handles.plot_pos,'Z'); %The axis label

% Displays the sound speed
global sound_speed;
if sound_speed == 0
    %the value of the sound speed has not been calibrated yet
    set(handles.sound_speed_value,'String','-');
    set(handles.sound_speed_value,'BackgroundColor',[1,0.4,0.4]);
else
    %there is a value for the sound speed already
    set(handles.sound_speed_value,'String',num2str(sound_speed));
    set(handles.sound_speed_value,'BackgroundColor',[0.3 0.75 0.3]);
end

% Displays the positions of the stationary modules at the table
set(handles.stationary_modules,'Data',stationary_modules_positions);

%Remove the visibility of the buttons that cannot be used, while there is
%no fixed module connected
set(handles.calib_ss_button, 'Visible', 'Off'); %calibration button
% set(handles.auto_meas_tbutton, 'Visible', 'Off'); %automatic measurements button
% set(handles.manual_read_pushbutton, 'Visible', 'Off'); %manual measurements button
 

set(handles.err_433, 'Visible', 'Off'); %error messages
set(handles.err_nrf, 'Visible', 'Off'); %error messages
 
%set(handles.fs_device , 'String', '/dev/ttyACM0');
