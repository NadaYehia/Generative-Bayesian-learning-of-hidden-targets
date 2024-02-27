function [posterior,flg,curr_t_lst_caching]=caching(reward_rate, current_posterior,As,Os,arena,clearnce,T,h1,h2,previous_t_lst_caching)


% calculate entropy of current posterior as measure of certainity
posterior_uncertainity=my_entropy(current_posterior);

% calculate the entropy of the reward time series: x is either 0 or 1. from
% time of last caching till this run
p_no_hit= numel( find(reward_rate(previous_t_lst_caching:end)==0))/ numel(reward_rate(previous_t_lst_caching:end));
p_hit= numel(   find(reward_rate(previous_t_lst_caching:end)==1))/ numel(reward_rate(previous_t_lst_caching:end));

curr_reward_entropy=my_entropy([ p_no_hit p_hit]);

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