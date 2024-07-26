%% transform circle in r, theta to x,y space
x01=pi/2;
y01=50;
h1=ezplot(@(x,y) ((x-x01)./(pi)).^2 + ((y-y01)./(1000)).^2 -(0.01^2),[0 pi 1 500]);
[yr,xth]=find(h1.ZData<=1e-5 );
xx=h1.XData;
yy=h1.YData;
% figure,scatter(xx(xth),yy(yr))
figure(3); hold on, scatter(yy(yr).*cos(xx(xth)),yy(yr).*sin(xx(xth)),'r')



x02=pi/4;
y02=200;
figure; hold on 
h2=ezplot(@(x,y) ((x-x02)./(pi)).^2 + ((y-y02)./(1000)).^2 -(0.01^2),[0 pi 1 500]);
hold off; 
[yr,xth]=find(h2.ZData<=1e-5 );
xx=h2.XData;
yy=h2.YData;
% figure,scatter(xx(xth),yy(yr))
figure(3); hold on,scatter(yy(yr).*cos(xx(xth)),yy(yr).*sin(xx(xth)),'r')


%% transform lines in r,theta space to x and y
n1=100;
n2=100;
theta_col=linspace(0,pi/2,n2)';
r_row=linspace(0,200,n1);
figure;
for i=1:n1
    hold on, plot(r_row.*cos(theta_col(i)),r_row.*sin(theta_col(i)),'r');
end

for i=1:n2
    hold on, plot(r_row(i).*cos(theta_col),r_row(i).*sin(theta_col),'b');
end




%% transform circle in x,y to r,theta

x01=-205;
y01=150;
h1=ezplot(@(x,y) ((x-x01)).^2 + ((y-y01)).^2 -(10^2),[-400 0 1 600]);
[ys,xs]=find(h1.ZData<=0 );
xx=h1.XData;
yy=h1.YData;
ps=[xx(xs);yy(ys)'];

% figure,scatter(xx(xth),yy(yr))
rs=vecnorm(ps);
thetas=atan2(yy(ys)',xx(xs));
figure(3); hold on, scatter(thetas,rs,'r')
xlim([0  pi])

