function varargout = interface(varargin)
% INTERFACE MATLAB code for interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interface

% Last Modified by GUIDE v2.5 03-May-2018 21:42:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interface is made visible.
function interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global CONNECT_FS;
global CONNECT_robot;
global dt;
CONNECT_FS=0;
CONNECT_robot=0;
dt = str2double(get(handles.dt_value,'String'));

% Finds at the UBUNTU system the name of the device for serial
% communication - FIXED STATION
try
    devices = strsplit(ls('/dev/ttyACM*'),'\n');
%     set(handles.fs_device , 'String', devices(1));
catch
     set(handles.fs_device , 'String', '');
     set(handles.button_comm_FS , 'Visible', 'Off');
     set(handles.auto_meas_tbutton , 'Visible', 'Off');
     set(handles.manual_read_pushbutton , 'Visible', 'Off');
     set(handles.calib_ss_button , 'Visible', 'Off');
end

% Finds at the UBUNTU system the name of the device for serial
% communication - robot TRANSMITTER
try 
    devices = strsplit(ls('/dev/ttyUSB*'),'\n');
    set(handles.robot_device , 'String', devices(1));
catch
    set(handles.robot_device , 'String', '');
    set(handles.button_comm_robot , 'Visible', 'Off');
end

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in button_comm_FS.
function button_comm_FS_Callback(hObject, eventdata, handles)
% hObject    handle to button_comm_FS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global CONNECT_FS;
global fixed_station_dev;
global END_EXECUTION;
[connect, send, receive, disconnect]=Serial_Comm;
if CONNECT_FS==0
    try
        fixed_station_dev=connect(get(handles.fs_device,'String'),57600);
        set(hObject,'String','DISABLE Comm. - Fixed Station');
        RGB=get(hObject,'BackgroundColor')/1.5;
        set(hObject,'BackgroundColor',RGB);
        set(handles.calib_ss_button, 'Visible', 'On'); %calibration button
        CONNECT_FS=1;
    catch exception
        disp(exception);
        CONNECT_FS=0;
        END_EXECUTION=1;
    end
else
    disconnect(fixed_station_dev);
    set(hObject,'String','ENABLE Comm. - Fixed Station');
    RGB=get(hObject,'BackgroundColor')*1.5;
    set(hObject,'BackgroundColor',RGB);
    CONNECT_FS=0;
end

function dt_value_Callback(hObject, eventdata, handles)
% hObject    handle to dt_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dt_value as text
%        str2double(get(hObject,'String')) returns contents of dt_value as a double
global dt;
dt = str2double(get(hObject,'String'));


% --- Executes during object creation, after setting all properties.
function dt_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dt_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in button_comm_robot.
function button_comm_robot_Callback(hObject, eventdata, handles)
% hObject    handle to button_comm_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global CONNECT_robot;
global robot_comm_dev;
global END_EXECUTION;

[connect, send, receive, disconnect]=Serial_Comm;
if CONNECT_robot==0
    try    
        robot_comm_dev=connect(get(handles.robot_device,'String'),57600);
        set(hObject,'String','DISABLE Comm. - Robot XBee');
        RGB=get(hObject,'BackgroundColor')/1.5;
        set(hObject,'BackgroundColor',RGB);
        CONNECT_robot=1;
    catch exception
        disp(exception);
        CONNECT_robot=0;
        END_EXECUTION=1;
    end
else
    disconnect(robot_comm_dev);
    set(hObject,'String','ENABLE Comm. - Robot XBee');
    RGB=get(hObject,'BackgroundColor')*1.5;
    set(hObject,'BackgroundColor',RGB);
    CONNECT_robot=0;
end



function fs_device_Callback(hObject, eventdata, handles)
% hObject    handle to fs_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fs_device as text
%        str2double(get(hObject,'String')) returns contents of fs_device as a double


% --- Executes during object creation, after setting all properties.
function fs_device_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fs_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function robot_device_Callback(hObject, eventdata, handles)
% hObject    handle to robot_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robot_device as text
%        str2double(get(hObject,'String')) returns contents of robot_device as a double


% --- Executes during object creation, after setting all properties.
function robot_device_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robot_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global END_EXECUTION;
END_EXECUTION=1;


% --- Executes when entered data in editable cell(s) in stationary_modules.
function stationary_modules_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to stationary_modules (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
global stationary_modules_positions;
stationary_modules_positions = get(hObject, 'Data');
num_stationary_modules = size(stationary_modules_positions,1);

%Begins the figure plot
cla(handles.plot_pos)
plot3(handles.plot_pos, stationary_modules_positions(1,1),stationary_modules_positions(1,2),stationary_modules_positions(1,3),'rx'); %the point
plot3(handles.plot_pos, [stationary_modules_positions(1,1) stationary_modules_positions(1,1)],[stationary_modules_positions(1,2) stationary_modules_positions(1,2)],[0 stationary_modules_positions(1,3)],'k','LineWidth',1.5); %the line to the ground
for i=2:num_stationary_modules
    plot3(handles.plot_pos, stationary_modules_positions(i,1),stationary_modules_positions(i,2),stationary_modules_positions(i,3),'rx');
    plot3(handles.plot_pos, [stationary_modules_positions(i,1) stationary_modules_positions(i,1)],[stationary_modules_positions(i,2) stationary_modules_positions(i,2)],[0 stationary_modules_positions(i,3)],'k','LineWidth',1.5);
end

% --- Executes on button press in manual_read_pushbutton.
function manual_read_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to manual_read_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global manual_single_read;
% Look which distances are selected
for i=1:10
    if (get(handles.(sprintf('d%d_checkbox', i)),'Value') == get(handles.(sprintf('d%d_checkbox', i)),'Max'))
        manual_single_read = [manual_single_read, i];
    end
end

% --- Executes on button press in auto_meas_tbutton.
function auto_meas_tbutton_Callback(hObject, eventdata, handles)
% hObject    handle to auto_meas_tbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of auto_meas_tbutton
button_state = get(hObject,'Value');
global AUTOMATIC_MEASUREMENTS;
if button_state == get(hObject,'Max')
    set(handles.uipanel6, 'Visible', 'Off');
    RGB=get(hObject,'BackgroundColor')/1.5;
    set(hObject,'BackgroundColor',RGB);
    AUTOMATIC_MEASUREMENTS=1;
elseif button_state == get(hObject,'Min')
    set(handles.uipanel6, 'Visible', 'On');
    RGB=get(hObject,'BackgroundColor')*1.5;
    set(hObject,'BackgroundColor',RGB);
    AUTOMATIC_MEASUREMENTS=0;
end


% --- Executes on button press in calib_ss_button.
function calib_ss_button_Callback(hObject, eventdata, handles)
% hObject    handle to calib_ss_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

button_state = get(hObject,'Value');
global CALIB_SOUND_SPEED;
global CONNECT_FS;
%CONNECT_FS=1;
if button_state == get(hObject,'Max')
    set(hObject, 'Visible', 'Off');
    CALIB_SOUND_SPEED=1;
end


% --- Executes on button press in d6_checkbox.
function d6_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to d6_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d6_checkbox


% --- Executes on button press in d7_checkbox.
function d7_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to d7_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d7_checkbox


% --- Executes on button press in d8_checkbox.
function d8_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to d8_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d8_checkbox


% --- Executes on button press in d10_checkbox.
function d10_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to d10_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d10_checkbox


% --- Executes on button press in d9_checkbox.
function d9_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to d9_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d9_checkbox
