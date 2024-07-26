function [mu, omega]=convert_xy_velo_angle_fixed_T(x_op,y_op,tol,T)
 
omega=[];
mu=[];
epsi=(tol-pi/2);

for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); 
    
    omega(i)=heading_offset-(pi/2);
    
    dx_=x_op(i);
    dy_=y_op(i);

    eucl_dist=sqrt(dx_^2 +dy_^2);

    mu_n=eucl_dist*((4*epsi)+(4*pi));
    mu_d= (pi*T)*sinc(epsi/pi);
    mu(i)= mu_n/mu_d;

   
    
end


end