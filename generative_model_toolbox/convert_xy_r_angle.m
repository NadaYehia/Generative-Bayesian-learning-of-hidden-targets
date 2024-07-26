function [r, omega]=convert_xy_r_angle(x_op,y_op)
 
omega=[];
r=[];

for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); 
    
    omega(i)=heading_offset-(pi/2);
    
    dx_=x_op(i);
    dy_=y_op(i);

    eucl_dist=sqrt(dx_^2 +dy_^2);

    
    r(i)= eucl_dist;

   
    
end


end