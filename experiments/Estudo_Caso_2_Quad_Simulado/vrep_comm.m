function [sys,x0,str,ts] = vrep_comm(t,x,u,flag)
% Dispatch the flag. The switch function controls the calls to
% S-function routines at each simulation stage.
switch flag,
	case 0
		[sys,x0,str,ts] = mdlInitializeSizes; % Initialization
		
	case 3
		sys = mdlOutputs(t,x,u); % Calculate outputs
	
	case 9
		sys = mdlTerminate(t,x,u);
	
	case { 1, 2, 4 }
		sys = []; % Unused flags
	
	otherwise
	error(['Unhandled flag = ',num2str(flag)]); % Error handling
end;
% End of function vrep_comm.


%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = []; % No continuous states
str = []; % No state ordering
ts = [-1 0]; % Inherited sample time - sample time: [period, offset]

% speicfy that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

global clientID;
global vrep;
% global quad_base;
% global targetObj;

if clientID==-1
	%Initiate the connection and simulation at Vrep
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
end

if (clientID>-1)
        disp('Connected to remote API server');	
			
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
		end
		
		%Starting the simulation in V-REP.
		[errorCodeStart]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        pause(5);
        disp('Simulation at V-REP initialized!')
end
% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u)
global clientID;
global vrep;
global quad_base;
global targetObj;

sys = [];
% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
global clientID;
global vrep;

% Now send some data to V-REP in a non-blocking fashion:
vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);

% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);

%stop the simulation at vrep
[errorCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
pause(3);
disp('Simulation terminated error Code');
disp(errorCode);

% Now close the connection to V-REP:    
vrep.simxFinish(clientID);

sys = [];

% end mdlTerminate