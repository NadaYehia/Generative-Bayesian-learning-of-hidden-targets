%% 
function [prior,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,arena,spc)

prior=ones(size(Ths,2),size(Rs,2));

 % mask the action space with the arena boundaries
[r_boundary,theta_boundary]=find_actions_bounds(arena,spc);

 % given the action boundary values, create a binary mask for the prior
[bw_boundary,theta_bounds,r_bounds]=convert_poly_to_mask(r_boundary,theta_boundary,size(prior),Rs,Ths);


 prior=prior.*bw_boundary;
 
 
 


end