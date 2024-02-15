function [mu, omega]=convert_xy_velo_angle(x_op,y_op)
 omega=[];
 mu=[];
 T=1000;
% scale factor of the area under the speed curve.
w=(2*pi)/(T);
t1=[0:(T/2)];
speed= [sin(w.*t1)]; 
k=(T)/pi;


for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); % this goes from -pi/2 to pi/2, as omega in our model.

    omega(i)=heading_offset-(pi/2);

    eucli_dist=sqrt((x_op(i)^2)+(y_op(i)^2));
    vmax_n=4*eucli_dist;
    mu(i)=vmax_n/pi;

       
end


end