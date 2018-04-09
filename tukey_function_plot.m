x = linspace(-30, 30, 500);
y = x.^2;
y( y>100 ) = 100;
plot(x,y)
ylim([0, max(y)+15]); grid on;