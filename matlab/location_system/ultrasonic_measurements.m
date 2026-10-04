function [ message ] = ultrasonic_measurements( n_emitters, handles, fixed_station_dev, method )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    [connect, send, receive, disconnect]=Serial_Comm;
    message = [];
    global working_receiver;

    if(strcmp(method,'all'))
        %Reads all the modules at once
        %-------------------------------
        us_step = tic;
        send(fixed_station_dev, 'UA!');
        for i = 1:n_emitters
            set(handles.(sprintf('d%d_value', i)),'BackgroundColor',[0.8 0.8 0.8]);
            receive(fixed_station_dev);
        end
        fprintf('US-[%.3f]sec\n', toc(us_step));
        nrf_step = tic;
        % 2.4GHz RF signal
        for i=1:length(working_receiver)
            message{i}=''; tries=1;
            while (length(message{i})<10) && (tries<3)
                if(tries>1)
                    disp('No message from receiver!!!!!');
                end
                send(fixed_station_dev,  strcat('D',num2str(working_receiver(i)),'!'));
                message{i} = receive(fixed_station_dev);
%                 disp(message{i});
%                 disp(length(message{i}));
                tries=tries+1;
            end
        end
        fprintf('NRF-[%.3f]sec\n', toc(nrf_step));
        %-------------------------------
    else
        %Reads the modules individually 
        %-------------------------------
        set(handles.err_433, 'Visible', 'Off'); %error messages
        set(handles.err_nrf, 'Visible', 'Off'); %error messages
        tStartModule = tic;
        i=1;
        while i<=n_emitters
            fprintf('%d',i);
            %How many 433MHz signals I want to send, before send an NRF msg
            % --- 1º Sig 433 ---
            set(handles.(sprintf('d%d_value', i)),'BackgroundColor',[0.8 0.8 0.8]);
            send(fixed_station_dev, strcat('U',num2str(i),'!'));
            receive(fixed_station_dev);
    
%             % --- 2º Sig 433 ---
%             i=i+1;
%             set(handles.(sprintf('d%d_value', i)),'BackgroundColor',[0.8 0.8 0.8]);
%             send(fixed_station_dev, strcat('U',num2str(i),'!'));
%             receive(fixed_station_dev);
    
            %I am measuring 2 dists before ask the receiver anything
    
            % 2.4GHz RF signal
            send(fixed_station_dev, strcat('D',num2str(working_receiver),'!'));
            tmp = receive(fixed_station_dev);
    
            try
                parsemsg_tmp = strsplit(tmp,{'-','.','\n'});
                if max(size(parsemsg_tmp))>2 %&& str2double(parsemsg{2})<15000
                    message=strcat(message,tmp);
                    i=i+1; %moving the next emitter module when the measurement was ok
                    %fprintf('-[%.3f]sec ', toc(tStartModule));
                    tStartModule = tic;
                else
                    % The code sent the info to read a given signal,
                    % but inside the receiver there is no value read,
                    % so there was a failure in RF 433 message
                    disp('ERROR RF433MHz');
                    set(handles.err_433, 'Visible', 'On'); %error messages
    
                end
            catch
                %parse (strsplit) fails when the message formated
                %wrongly, meaning NRF radio error
                disp(strcat('MSG ERROR [',tmp,'] [',num2str(max(size(parsemsg_tmp))),']'));
                set(handles.err_nrf, 'Visible', 'On'); %error messages
            end
    
            Tmodule=toc(tStartModule);
            %fprintf('-[%.3f]sec ', Tmodule);
    
            %If I already spent more then 120ms trying to read this module, go to the next one
            if Tmodule>0.1
                fprintf('Giving up [%d] - [%.3f]sec\n', i,Tmodule);
                i=i+1; %moving the next emitter module when the measurement took to long
                tStartModule = tic;
            end
        end
        %-------------------------------
    end
end

