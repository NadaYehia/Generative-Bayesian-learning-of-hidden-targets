
function [r_anchors,theta_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Ths,dist_criterion,theta_bounds,...
                                                                   r_bounds,r_home,radii_noise_offset,...
                                                                   radii_noise_slope,angular_noise,min_local_diff,pr_thresh,roi_size)

% this function samples anchors from the current posterior, it can jitter
% them with noise (if applicable) then outputs them for the trajectory
% planner 

if(strcmp(sampler,'proportional'))

    [r_anchors,theta_anchors,anchors_no]=anchors_prop_sampler_wnn_merge (posterior,initial_ancs,Rs,Ths,dist_criterion);
else
    [r_anchors,theta_anchors,anchors_no]=anchors_peak_sampler(posterior,Rs,Ths,initial_ancs,min_local_diff,pr_thresh,roi_size);
 
end

% if there is any input noise:
if(radii_noise_offset || radii_noise_slope ||angular_noise)
    % add sampling noise:
       for f=1:anchors_no
    
        theta_anchors(f)= theta_anchors(f) +(randn(1)*angular_noise);
        
        scale= radii_noise_offset+radii_noise_slope*r_anchors(f);
        r_anchors(f)= r_anchors(f)+(scale*randn(1));
    
       end
    
    % Confine the noisy anchors to within the arena enclosure. 
    
    r_anchors(r_anchors<r_home)=r_home;
    theta_anchors(theta_anchors>(pi/2))=pi/2;
    theta_anchors(theta_anchors<(-pi/2))=-pi/2;
     
    % for theta anchors <pi/2 & >-pi/2, confine the noisy anchors radii
    % to the maximum allowed by the arena walls.
    
    for f=1:anchors_no
    
        [~,thetas_minus_anchor_sorted]= sort(abs(theta_anchors(f) -Ths(theta_bounds)));
        
        %closest 2 angle bins to the anchor angle
        indx=thetas_minus_anchor_sorted(1:2);
        
    
        % set the maximum allowed radius around this theta 
        % to the minimum of the radius bounds from the closest 2 angle bounds bins 
        %  
        max_r_this_theta=min(Rs(r_bounds(indx)));
    
        if(r_anchors(f)>max_r_this_theta)
            r_anchors(f)=max_r_this_theta;
        end
    
     
    end

end

end
