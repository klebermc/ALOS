function [ x , y , z ] = round_TS( fixed_modules_positions , distance_readings)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    firt_guess = [0,0,0];
    x=[0,0,0];
    numMeas = length(distance_readings);
    
    historyX=[];
    historyY=[];
    historyZ=[];
    
    N_round=4;
    
    %round with 4 readings
    while N_round<numMeas
        partial_FMpos=fixed_modules_positions;
        partial_readings=distance_readings;
        
        %choose N_round measurements to make a calculation
        while length(partial_readings)>N_round
            j = round(1 + (length(partial_readings)).*rand(1,1));
            aux1=[];
            aux2=[];
            for k=1:(length(partial_readings))
                if k<j
                    aux1=[aux1;partial_FMpos(k,:)];
                end
                if k>j
                    aux2=[aux2;partial_FMpos(k,:)];
                end
            end
            partial_FMpos=[aux1;aux2];
            aux1=[];
            aux2=[];
            for k=1:(length(partial_readings))
                if k<j
                    aux1=[aux1;partial_readings(k)];
                end
                if k>j
                    aux2=[aux2;partial_readings(k)];
                end
            end
            partial_readings=[aux1;aux2];
        end
        if abs(max(partial_FMpos(:,3)) - min(partial_FMpos(:,3))) > 0.20
            %Making sure measurements from modules 2, 4, 6 or 8 were choosen
            %can run taylor series
            [x,y,z]=taylor_series( partial_FMpos , partial_readings, firt_guess );
            historyX=[historyX,x];
            historyY=[historyY,y];
            historyZ=[historyZ,z];
        end
        if rem(length(historyX),1000) == 0
            N_round=N_round+1;
        end
    end
    
    [x,y,z]=taylor_series( fixed_modules_positions , distance_readings, firt_guess );
    figure; plot(historyX);
    figure; histogram(historyX);
    [x, mean(historyX), median(historyX)]
    [y, mean(historyY), median(historyY)]
    [z, mean(historyZ), median(historyZ)]
    x=mean(historyX);y=mean(historyY);z=mean(historyZ);
end

