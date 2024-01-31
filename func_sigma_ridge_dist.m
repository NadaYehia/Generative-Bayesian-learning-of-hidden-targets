function dist=func_sigma_ridge_dist(k,sigma_ridge,As,Os)


increment_x=2*k*sigma_ridge;
increment_y= (2*pi/1000)*k*sigma_ridge;

% D1=[0 0;  0 increment_y;  0 increment_y; 0 0];
D1=[0 0; increment_x 0; increment_x 0; 0 0];


D_= (D1)*[1/(std(As)^2) 0; 0  1/(std(Os)^2)]*(D1');

D=sqrt(reshape(diag(D_),[2 2]));
D(D==0)=nan;

dist=D(1,2);







end