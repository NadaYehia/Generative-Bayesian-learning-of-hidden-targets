
function [mu_anchors,omega_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds)


if(strcmp(sampler,'proportional'))

    [mu_anchors,omega_anchors,anchors_no]=anchors_prop_sampler_wnn_merge (posterior,initial_ancs,As,Os,dist_criterion);
else
    [mu_anchors,omega_anchors,anchors_no(k)]=anchors_peak_sampler(posterior,initial_ancs,As,Os,r_bounds,c_bounds);
 
end

end