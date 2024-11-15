%% 
function [prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,arena)

prior=ones(size(Os,2),size(Rs,2));

 % mask the action space with the arena boundaries
[r_boundary,omega_boundary]=find_actions_bounds(arena);

 % given the action boundary values, create a binary mask for the prior
[bw_boundary,r_bounds,c_bounds]=convert_poly_to_mask(r_boundary,omega_boundary,size(prior),Rs,Os);


 prior=prior.*bw_boundary;
 
 
 


end