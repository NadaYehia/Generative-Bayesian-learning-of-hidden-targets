%% 
function [prior,r_bounds,c_bounds]= set_control_actions_space(As,Os,arena)

prior=ones(size(Os,2),size(As,2));

 % mask the action space with the arena boundaries
[mu_boundary,omega_boundary]=find_actions_bounds(arena);

 % given the action boundary values, create a binary mask for the prior
[bw_boundary,r_bounds,c_bounds]=convert_poly_to_mask(mu_boundary,omega_boundary,size(prior),As,Os);


 prior=prior.*bw_boundary;
 prior=prior./(sum(sum(prior)));


end