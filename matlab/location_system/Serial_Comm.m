function [connect, send, receive, disconnect]=Serial_Comm
connect = @connect_to_fixed_module;
send = @send_message;
receive = @receive_message;
disconnect = @disconnect_to_fixed_module;


function serial_obj = connect_to_fixed_module(device_name,BaudRate)
	serial_obj = serial(device_name,'BaudRate',BaudRate,'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 0.2);
	fopen(serial_obj);

function send_message(serial_obj, message)	
%     disp(strcat('Sending msg: [',message,']'));
% 	fprintf(serial_obj, message);
    fprintf(serial_obj, '%s', message);
    flushoutput(serial_obj)
	

function in_message = receive_message(serial_obj)
	in_message = fscanf(serial_obj);
    flushinput(serial_obj)
	

function disconnect_to_fixed_module(serial_obj)
	fclose(serial_obj);
	delete(serial_obj);
	clear serial_obj;
