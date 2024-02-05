function [mu, omega]=convert_xy_velo_angle(x_op,y_op,tol)
 omega=[];
 mu=[];
 T=1000;
% scale factor of the area under the speed curve.
w=(2*pi)/(T);
t1=[0:(T/2)];
speed= [sin(w.*t1)]; 

k=sum(speed);

% figure;
for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); % this goes from -pi/2 to pi/2, as omega in our model.

    omega(i)=heading_offset-(pi/2);

    r1=k*y_op(i)* ( (pi^2)-(4* (tol^2)) );
    q1= pi*T*(cos(tol)*sin(heading_offset));
    mu_y= r1/q1;

    r=k*x_op(i)* ( (pi^2)-(4* (tol^2)) );
    q= pi*T*(cos(tol)*cos(heading_offset));
    mu_x= r/q;

    mu(i)=mu_x;

    if (isinf(abs(mu(i))))
        mu(i)=mu_y;
    
    end
  
    
end


end