
function [r_anchors,theta_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Ths,dist_criterion,theta_bounds,...
                                                                   r_bounds,r_home,radii_noise_offset,...
                                                                   radii_noise_slope,angular_noise,roi_size)
% Sampling_next_actions - Samples anchor points for the next actions based on the posterior distribution.
%                         Optionally adds noise to the sampled anchors and ensures they stay within the arena bounds.
%
% Inputs:
%   posterior              - 2D matrix representing the posterior distribution over actions.
%   sampler                - Sampling method: 'proportional' or another method (e.g., 'peak').
%   initial_ancs           - Initial number of anchor points to sample.
%   Rs                     - Vector of possible values for the radial dimension (r).
%   Ths                    - Vector of possible values for the angular dimension (theta).
%   dist_criterion         - Distance threshold for merging anchor points (used in proportional sampling).
%   theta_bounds           - Indices of theta values that define the arena's angular bounds.
%   r_bounds               - Indices of radial values that define the arena's radial bounds.
%   r_home                 - Minimum allowed radial value (home position).
%   radii_noise_offset     - Base noise level for radial noise.
%   radii_noise_slope      - Slope of noise scaling with radial distance.
%   angular_noise          - Noise level for angular noise.
%   roi_size               - Size of the region of interest (used in peak sampling).
%
% Outputs:
%   r_anchors              - Sampled (and possibly noisy) radial coordinates of anchor points.
%   theta_anchors          - Sampled (and possibly noisy) angular coordinates of anchor points.
%   anchors_no             - Number of anchor points after sampling and merging.

%%

 % Sample anchor points based on the chosen sampling method.
if(strcmp(sampler,'proportional'))
    % Use proportional sampling with nearest neighbor merging.
    [r_anchors,theta_anchors,anchors_no]=anchors_prop_sampler_wnn_merge (posterior,initial_ancs,Rs,Ths,dist_criterion);
else
    % Use peak sampling (e.g., sampling from peaks of the posterior).
    [r_anchors,theta_anchors,anchors_no]=anchors_peak_sampler(posterior,Rs,Ths,initial_ancs,roi_size);
 
end


%% 
% 
% % If any noise parameters are provided, add noise to the sampled anchors.

if(radii_noise_offset || radii_noise_slope ||angular_noise)
    % Loop through each anchor point and add noise.

       for f=1:anchors_no
    
        % Add angular noise (Gaussian-distributed).   
        theta_anchors(f)= theta_anchors(f) +(randn(1)*angular_noise);
        
        % Add radial noise (scaled by the anchor's radial distance)
        scale= radii_noise_offset+radii_noise_slope*r_anchors(f);
        r_anchors(f)= r_anchors(f)+(scale*randn(1));
    
       end
    
    % Confine the noisy anchors to within the arena bounds.
    % Ensure radial values are not smaller than the home position.
    
    r_anchors(r_anchors<r_home)=r_home;

    % Ensure angular values are within the range [-pi/2, pi/2].
    theta_anchors(theta_anchors>(pi/2))=pi/2;
    theta_anchors(theta_anchors<(-pi/2))=-pi/2;
     
    % For each anchor, ensure the radial value does not exceed the maximum allowed
    % by the arena walls at the corresponding angle.
    
    for f=1:anchors_no
        
        % Find the closest two angle bins to the anchor's angle.
        [~,thetas_minus_anchor_sorted]= sort(abs(theta_anchors(f) -Ths(theta_bounds)));
        
        indx=thetas_minus_anchor_sorted(1:2); % Indices of the closest two angle bins.
        
    
        % Set the maximum allowed radius for this angle to the minimum of the
        % radial bounds at the closest two angle bins. 

        max_r_this_theta=min(Rs(r_bounds(indx)));
    
        % If the anchor's radial value exceeds the maximum allowed, clip it.
        
        if(r_anchors(f)>max_r_this_theta)
            r_anchors(f)=max_r_this_theta;
        end
    
     
    end

end

end
