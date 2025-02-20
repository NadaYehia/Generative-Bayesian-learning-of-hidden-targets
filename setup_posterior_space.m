function [prior_flat,theta_bounds,r_bounds,r_after_home,arena_home_mask] = setup_posterior_space(Rs,Ths,env,radius_around_home)

[prior_flat,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,env.arena_dimensions);
prior_flat=boolean(prior_flat);

[prior_exc_home,c_home]=semi_circle_around_home(prior_flat,radius_around_home,Rs);
r_after_home=Rs(c_home+1);

prior_flat=prior_flat&prior_exc_home;
arena_home_mask=prior_flat;
naned_arena_home_mask=1-arena_home_mask;
naned_arena_home_mask(find(naned_arena_home_mask))=nan;
arena_home_mask=arena_home_mask+naned_arena_home_mask;
prior_flat=arena_home_mask./(nansum(arena_home_mask(:)));



end