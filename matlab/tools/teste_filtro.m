close all;
grid on;
hold on;
plot(vect,'DisplayName','posx');
num=0.3935;den=[1, -0.6065];

for i=11:max(size(vect))
    teste=vect(i-10:i);
    filtered_quad_pos = filter(num,den,teste);
    plot(i,filtered_quad_pos(11),'rx')
end
hold off;