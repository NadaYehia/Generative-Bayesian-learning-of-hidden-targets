function [posterior,flg,curr_t_lst_caching]=caching_w_bayes_on_reward_prob(reward_rate,current_posterior,As,Os,arena,clearnce,...
                                            T,threshold,previous_t_lst_caching)

    
    curr_rew=reward_rate(end); % the current outcome being 0 or 1

    P_ri= numel(find(reward_rate==curr_rew))./ numel(reward_rate);

    % the likelihood is the posterior action distribution
    if(curr_rew==0) % no hit
        %how likely is choosing these current actions given there is no reward.



    else

        
    end










end