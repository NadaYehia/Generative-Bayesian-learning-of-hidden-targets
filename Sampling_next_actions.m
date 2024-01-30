
function [mu_anchors,omega_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds)


if(strcmp(sampler,'proportional'))

    [mu_anchors,omega_anchors,anchors_no]=anchors_sampler_nn_merge (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
else
    [mu_anchors,omega_anchors,anchors_no(k)]=anchors_sampler_MuliPeakPost (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
 
end

end