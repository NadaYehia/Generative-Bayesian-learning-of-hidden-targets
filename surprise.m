function [flg,Sw,Sf]=surprise(L1,outcome, current_posterior,h0,arena_home_mask)
% this function computes the surprise associated with an outcome given a likelihood, 
% under the current posterior and a flat prior. First, it calculates the
% marginal probability of observing an outcome as follows,
% P(outcome)= sum_{over all actions} [L*posterior]. Then it uses 
% log (P(outcome)) as the measure of surprise in a given outcome; the lower
% the outcome probability is, the more surprising it should be, and the higher is the probability that the
% environment might have changed. It means the observed outcome is not consistent
% with the current posterior probability distribution any longer.
% Inputs:
%   L1: Likelihood matrix.
%   outcome: Binary outcome (0 or 1). If 0, the likelihood is inverted (1 - L1).
%   current_posterior: Current posterior probability distribution.
%   h0: Threshold for surprise.
%   arena_home_mask: Mask representing a flat prior over the valid target locations in the arena.
% Outputs:
%   flg: Flag indicating whether the surprise exceeds the threshold (h0).
%   Sw: Surprise based on the current posterior.
%   Sf: Surprise based on the flat prior.

%%
% If the outcome is 0, invert the likelihood (1 - L1)
if(outcome==0)
 
    L1=(1-L1);
end

% Compute the flat prior by normalizing the arena_home_mask
flat_prior=arena_home_mask./(nansum(arena_home_mask(:)));

% Compute the probability of the outcome under the flat prior
p_flat_2d= (flat_prior.*L1);
p_flat=nansum(p_flat_2d(:)); % Sum over all elements

% Compute the probability of the outcome under the current posterior
p_working_2d=current_posterior.*L1;
p_working=nansum(p_working_2d(:)); % Sum over all elements

% Compute the surprise values
Sw=-log10(p_working); % Surprise based on the current posterior
Sf=-log10(p_flat); % Surprise based on the flat prior

% Determine if the surprise based on the current posterior 
% exceeds the threshold
flg=Sw>h0; % Flag is true if Sw exceeds h0


end