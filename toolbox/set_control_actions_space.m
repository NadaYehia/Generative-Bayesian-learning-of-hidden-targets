function [prior,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,arena)
% this function maps the arena bounds in the Cartesian space to the polar coordinates space.
% Input: R, Thetas domain values: Rs, Ths
%        arena: arena dimensions centered about the (x,y) Home location which is (W/2,0), 
%               X-coordinates shifted to the left by half the arena width.
 
%%
prior=ones(size(Ths,2),size(Rs,2));
% rotate the thetas to be relative to the +ve x-axis
Ths_transformed=Ths+(pi/2);

%% find the angles in the posterior discretization that maps the angles spanning the right
% vertical wall of the arena: from bottom right corner to top right corner
right_vertical_wall=[atan2(arena(3),arena(2)),...
    atan2(arena(4),arena(2))];
right_vertical_wall_binned=Ths_transformed(Ths_transformed>=right_vertical_wall(1) & Ths_transformed<=right_vertical_wall(2));
% Polar transformation of a vertical line in Cartesian space: x=constant,
% x= r cos(theta);
% C= r cos(theta), where C is the second x-coord (x2) of the arena
% dimensions: 
f1= (arena(2)./cos(right_vertical_wall_binned));

%% left vertical wall of the arena: from top left to bottom left corner.
left_vertical_wall=[atan2(arena(4),arena(1)),...
    atan2(arena(3),arena(1))];
left_vertical_wall_binned=Ths_transformed(Ths_transformed>=left_vertical_wall(1) & Ths_transformed<=left_vertical_wall(2));
f3= (arena(1)./cos(left_vertical_wall_binned));

%% top horizontal wall of the arena: from top right corner to top left corner
top_horizontal_wall=[atan2(arena(4),arena(2)),...
    atan2(arena(4),arena(1))];

top_horizontal_wall_binned=Ths_transformed(Ths_transformed>=top_horizontal_wall(1) & Ths_transformed<=top_horizontal_wall(2));

f2=(arena(4)./sin(top_horizontal_wall_binned));

%% Finding the bin in the radial domain (Rs) mapping the bottom right and bottom left corners of the arena 
% [bottom horizontal wall] 
  
max_x_wall=floor( arena(2)/(Rs(2)-Rs(1)) )+1;

%% given the polar coordinates boundary values, create a binary mask for the prior
[bw_boundary,theta_bounds,r_bounds]=convert_poly_to_mask([f1 f2 f3  ],[right_vertical_wall_binned top_horizontal_wall_binned left_vertical_wall_binned ]-(pi/2),size(prior),Rs,Ths,max_x_wall);


 prior=prior.*bw_boundary;
 

end