function dist=func_sigma_ridge_dist(k,sigma_ridge)

% first find the equivalent distance to one sigma in both x&y around a
% point in the action space
 As=[100:2:800];
 Os=[-pi/2:2*pi/1000:pi/2];

 x0= 200;
y0=-0.5;

increment_x=2*k*sigma_ridge;
increment_y= (2*pi/1000)*k*sigma_ridge;

D1=[0 0; increment_x increment_y; increment_x increment_y; 0 0];

D_= (D1)*[1/(std(As)^2) 0; 0  1/(std(Os)^2)]*(D1');

D=sqrt(reshape(diag(D_),[2 2]));
D(D==0)=nan;

dist=D(1,2);







end