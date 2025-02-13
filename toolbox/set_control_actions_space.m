%% 
function [prior,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,arena)

prior=ones(size(Ths,2),size(Rs,2));

Ths_transformed=Ths+(pi/2);

theta1=[atan2(arena(3),arena(2)),...
    atan2(arena(4),arena(2))];

theta1_binned=Ths_transformed(Ths_transformed>=theta1(1) & Ths_transformed<=theta1(2));

f1= (arena(2)./cos(theta1_binned));

%
theta3=[atan2(arena(4),arena(1)),...
    atan2(arena(3),arena(1))];

theta3_binned=Ths_transformed(Ths_transformed>=theta3(1) & Ths_transformed<=theta3(2));

f3= (arena(1)./cos(theta3_binned));

%
theta2=[atan2(arena(4),arena(2)),...
    atan2(arena(4),arena(1))];

theta2_binned=Ths_transformed(Ths_transformed>=theta2(1) & Ths_transformed<=theta2(2));

f2=(arena(4)./sin(theta2_binned));

max_x_wall=ceil( arena(2)/(Rs(2)-Rs(1)) );
 % given the action boundary values, create a binary mask for the prior
[bw_boundary,theta_bounds,r_bounds]=convert_poly_to_mask([f1 f2 f3  ],[theta1_binned theta2_binned theta3_binned ]-(pi/2),size(prior),Rs,Ths,max_x_wall);


 prior=prior.*bw_boundary;
 
 
 


end