function [posterior,flg,curr_t_lst_caching]=caching_2(reward_rate, current_posterior,As,Os,arena,clearnce,T,h1,h2,...
    previous_t_lst_caching,n)


% calculate entropy of current posterior as measure of certainity
posterior_uncertainity=my_entropy(current_posterior);

% calculate the entropy of the reward time series: x is either 0 or 1.
% for n samples before the current time step

if (numel(reward_rate)-n>=1)
    p_no_hit= numel( find(reward_rate(end-n:end)==0))/ numel(reward_rate(end-n:end));
    p_hit= numel(   find(reward_rate(end-n:end)==1))/ numel(reward_rate(end-n:end));
    curr_reward_entropy=my_entropy([ p_no_hit p_hit]);
else 
    curr_reward_entropy=0;
end


max_entropy_reward_rate=1; % this is the maximum entropy you can get from a random binary variable

% 
[flat_prior,~,~]=set_control_actions_space(As,Os,arena,clearnce,T);
flat_prior_uncertainity= my_entropy(flat_prior);

% h1: hyperparameter for uncertainity estimation relative to flat prior.
% h2: hyperparameter for increase in reward rate entropy relative to the maximum 
%     entropy of a binary variable

if( (posterior_uncertainity/flat_prior_uncertainity) <h1 && curr_reward_entropy/max_entropy_reward_rate>h2)
    flg=1;
    curr_t_lst_caching=numel(reward_rate); % trial when caching happened.
    posterior= flat_prior;
else  % else, don't cache
    flg=0;
    curr_t_lst_caching=previous_t_lst_caching;
    posterior=current_posterior;
end


end