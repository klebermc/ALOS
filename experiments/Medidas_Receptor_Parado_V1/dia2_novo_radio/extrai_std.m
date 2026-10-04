clear
hold on
iter=1;
for p=1:9
    for h=0:3
        file2open = strcat('ponto',num2str(p),'_altura',num2str(h),'.mat');
        load(file2open);
        disp(file2open);
        %disp([std(module_position_lms(:,1)) std(module_position_lms(:,2)) std(module_position_lms(:,3))]*100)
        disp([std(module_position(:,1)) std(module_position(:,2)) std(module_position(:,3))]*100)
        %disp([std(module_position_aml(:,1)) std(module_position_aml(:,2)) std(module_position_aml(:,3))]*100)
        %if h==1 ; subplot(3,3,p); histogram(module_position_taylor(:,3)); end
        plot(iter,std(module_position(:,1)),'rx')
        plot(iter,std(module_position(:,2)),'bx')
        plot(iter,std(module_position(:,3)),'kx')
        iter=iter+1;
        
        clear('availability_history','module_position_lms','module_position_aml','module_position_taylor', 'measured_time_history', 'sound_speed', 'stationary_modules_positions');
    end
end