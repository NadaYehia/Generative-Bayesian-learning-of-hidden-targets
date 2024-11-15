
function [mu_anchors,omega_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,As,Os,dist_criterion,lin_idx_bounds,percentile)


if(strcmp(sampler,'proportional'))

    [mu_anchors,omega_anchors,anchors_no]=anchors_prop_sampler_wnn_merge (posterior,initial_ancs,As,Os,dist_criterion,lin_idx_bounds);
else
    [mu_anchors,omega_anchors,anchors_no]=anchors_peak_sampler(posterior,initial_ancs,As,Os,lin_idx_bounds,percentile,dist_criterion);
 
end

end