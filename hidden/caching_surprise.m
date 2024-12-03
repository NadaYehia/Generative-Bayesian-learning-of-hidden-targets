function [flg]=caching_surprise(outcome, current_posterior,current_actions,Rs,Os,h0,...
    env,min_radius_around_home)
    
% max a posteriori of p(actions) in working memory
for ac=2:size(current_actions,1)-1

    [~,omega_acs_idx]= min(abs(Os -current_actions(ac,1)) );
    [~,r_acs_idx]=min( abs(Rs-current_actions(ac,2)) );
    pr_post(ac)=current_posterior(omega_acs_idx,r_acs_idx);

end
pr_post(1)=[];
p_working=max(pr_post);

% max a posteriori of p(actions) in flat prior
[flat_prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,env.arena_dimensions);
[prior_exc_home,r_home,c_home]=semi_circle_around_home(flat_prior,min_radius_around_home,Rs,Os);
flat_prior=flat_prior.*prior_exc_home;
flat_prior=flat_prior./(sum(flat_prior(:)));

for ac=2:size(current_actions,1)-1
    [~,omega_acs_idx]= min(abs(Os -current_actions(ac,1)) );
    [~,r_acs_idx]=min( abs(Rs-current_actions(ac,2)) );
    pr_flat(ac)=flat_prior(omega_acs_idx,r_acs_idx);

end
pr_flat(1)=[];
p_flat=max(pr_flat);

% set surprise flag to 1 if Sw/Sf>h0
% outcome dependent surprise. 

if (outcome==1) % reward=1
    Sw=-log2(p_working);
    Sf=-log2(p_flat);
    flg= ((Sw/Sf)>h0);

else % reward =0
    Sw=-log2(1-p_working);
    Sf=-log2(1-p_flat);
    flg= ((Sw/Sf)>h0);

end



end