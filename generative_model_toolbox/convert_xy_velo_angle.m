function [mu, omega]=convert_xy_velo_angle(x_op,y_op,tol)
 
omega=[];
mu=[];
T=1000;
k=(T)/pi;
epsi=(tol-pi/2);
for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); 
    
    omega(i)=heading_offset-(pi/2);
    
    dx_=x_op(i);
    dy_=y_op(i);

    eucl_dist=sqrt(dx_^2 +dy_^2);
    mu_n=eucl_dist*((4*epsi)+(4*pi));
    mu_d= (pi^2)*sinc(epsi/pi);
    mu(i)= mu_n/mu_d;

   
    
end


end