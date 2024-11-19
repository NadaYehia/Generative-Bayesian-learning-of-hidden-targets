function [active_prior,maps_exh]=caching_routine(prior,...
    maps_exh,n_maps,outcome,current_actions,Rs,Os,env,min_radius_around_home)



    [flat_prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,env.arena_dimensions);
    [prior_exc_home,r_home,c_home]=semi_circle_around_home(flat_prior,min_radius_around_home,Rs,Os);
    flat_prior=flat_prior.*prior_exc_home;
    flat_prior=flat_prior./(sum(sum(flat_prior)));
    active_prior=flat_prior;
    maps_exh=0;




      
end
