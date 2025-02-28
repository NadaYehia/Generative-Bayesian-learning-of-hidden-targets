function [posterior]=Bayes_update_for_actions_params...
    (L1,target_hit,prior)
% Bayes_update_for_actions_params - Updates the prior belief using Bayes' 
% rule based on whether the target was hit or not.
% Inputs:
%   L1          - Likelihood of the target being hit given the current parameters.
%   target_hit  - Binary indicator (1 if target was hit, 0 otherwise).
%   prior       - Prior belief distribution over the parameters.
%
% Output:
%   posterior   - Updated belief distribution over the parameters after observing the target hit/miss.


if (target_hit==1)
     %% Equations 9 and (10.a)
     % If the target was hit, update the posterior using the likelihood L1.   
     posterior=(L1).*(prior); % Multiply prior by likelihood L1.
     posterior=posterior./( nansum(posterior(:)) ); % Normalize the posterior to sum to 1.
        
         
else
    %% Equations 9 and (10.b)
     % If the target was missed, update the posterior using the complement 
     % of the likelihood (1 - L1).    
     posterior=(1-L1).*prior; % Multiply prior by (1 - L1).
     posterior=posterior./( nansum(posterior(:)) ); % Normalize the posterior to sum to 1.
        
          
end
  

end