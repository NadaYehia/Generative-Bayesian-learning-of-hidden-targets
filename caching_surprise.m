function [posterior,flg]=caching_surprise(reward_rate, current_posterior,current_actions,As,Os,arena,clearnce,T,h0)


% p(r=1|current actions, and belief)
% convert mu_anchors and omega_anchors to r,c indexes in the posterior matrix
for ac=2:size(current_actions,1)-1

    omega_acs_idx= find(Os ==current_actions(ac,1));
    mu_acs_idx=find( As== current_actions(ac,2) );
    pr_post(ac)=current_posterior(omega_acs_idx,mu_acs_idx);

end
pr_post(1)=[];
avg_pr_post=mean(pr_post);

[flat_prior,~,~]=set_control_actions_space(As,Os,arena,clearnce,T);

% same but for the flat prior, p(r=1|current actions, and no belief)
for ac=2:size(current_actions,1)-1

    omega_acs_idx= find(Os ==current_actions(ac,1));
    mu_acs_idx=find( As== current_actions(ac,2) );
    pr_flat_prior(ac)=flat_prior(omega_acs_idx,mu_acs_idx);

end
pr_flat_prior(1)=[];
avg_pr_flat_prior=mean(pr_flat_prior);

if( ( -log(avg_pr_flat_prior)/-log(avg_pr_post)     ) >h0 && reward_rate(end)==0)
    
    flg=1;
    posterior= flat_prior;
else  % else, don't cache
    flg=0;
    posterior=current_posterior;
end


end