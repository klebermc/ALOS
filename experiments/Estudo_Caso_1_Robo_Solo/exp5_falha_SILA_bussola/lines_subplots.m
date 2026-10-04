% two 2x5 arrays with random data
a1 = rand(2,5);
a2 = rand(2,5);

% two subplots
subplot(211)
scatter(a1(1,:),a1(2,:))
% Convert axes coordinates to figure coordinates for 1st axes
xa1 = [100 200 300 400]/1000
ya1 = [100 100 100 100]/1000


subplot(212)
scatter(a2(1,:),a2(2,:))
% Convert axes coordinates to figure coordinates for 2nd axes
xa2 = [100 200 300 400]/1000
ya2 = [400 500 600 700]/1000

% draw the lines
for k=1:numel(xa1)
    annotation('line',[xa1(k) xa2(k)],[ya1(k) ya2(k)],'color','r');
end