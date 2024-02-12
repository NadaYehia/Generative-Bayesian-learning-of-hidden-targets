function [mu, omega]=convert_xy_velo_angle(x_op,y_op,tol)
 
omega=[];
mu=[];
T=1000;
double heading_offset;
qn=(pi*cos(tol));
qd=( ((pi^2)/4)-(tol^2) );
q=qn/qd; 

k=(T)/pi;

for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); % this goes from -pi/2 to pi/2, as omega in our model.
    
    omega(i)=heading_offset-(pi/2);

    r1=(T*cos(heading_offset))/4;
    mu_x= (k*x_op(i))/(r1*q);

    r=  (T*sin(heading_offset))/4;
    mu_y= (k*y_op(i)) /(r*q);


    mu(i)=mu_y;

   
  
    
end


end