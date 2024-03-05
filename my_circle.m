function h = my_circle(x,y,r)
hold on
th = 0:pi/8:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off