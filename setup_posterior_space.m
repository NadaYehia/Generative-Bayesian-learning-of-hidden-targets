function [prior_flat,theta_bounds,r_bounds,r_after_home,arena_home_mask] = setup_posterior_space(Rs,Ths,env,radius_around_home)

% map the arena bounds in Cartesian space to Polar space (R,theta).
[prior_in_arena,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,env.arena_dimensions);

% create binary mask with ones everywhere and 0s up till the distance
% considered as Home.
[~,c_home]=min(abs(Rs-radius_around_home));
prior_minus_home=ones(size(prior_in_arena));
prior_minus_home(:,1:c_home)=0;
r_after_home=Rs(c_home+1);

% find regions of the posterior that are within the arena enclosure AND
% outside home area
prior_in_arena_minus_home=prior_in_arena & prior_minus_home;
% add nans in the locations outside the arena bounds AND inside the home area 
arena_home_mask=double(prior_in_arena_minus_home);
arena_home_mask(arena_home_mask==0)=nan;
%normalize the prior to sum to 1
prior_flat=arena_home_mask./(nansum(arena_home_mask(:)));

end