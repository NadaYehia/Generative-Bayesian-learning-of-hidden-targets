
function [r_anchors,theta_anchors,anchors_no]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Os,dist_criterion,theta_bounds, r_bounds,r_home,percentile,...
    bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale)


if(strcmp(sampler,'proportional'))

    [r_anchors,theta_anchors,anchors_no]=anchors_prop_sampler_wnn_merge (posterior,initial_ancs,Rs,Os,dist_criterion,lin_idx_bounds);
else
    [r_anchors,theta_anchors,anchors_no]=anchors_peak_sampler(posterior,initial_ancs,Rs,Os,percentile);
 
end



%% add sampling noise:

for f=1:anchors_no

    theta_anchors(f)= theta_anchors(f) +(randn(1)*bestNormalFitAngleScale);
    
    scale= bestFitDataOffset+bestFitMeanToScaleRatio*r_anchors(f);
    r_anchors(f)= r_anchors(f)+(scale*randn(1));

end

%% Confine the noisy to within the arena enclosure. 

r_anchors(r_anchors<r_home)=r_home;
theta_anchors(theta_anchors>(pi/2))=pi/2;
theta_anchors(theta_anchors<(-pi/2))=-pi/2;

% for theta anchors <pi/2 & >-pi/2, confine the maximum radius
% to the maximum radius (arena bound) at this angle
for f=1:anchors_no

    theta_minus_bounds= abs(theta_anchors(f) -Os(theta_bounds));
    [minvalue]=min(theta_minus_bounds );
    
   [indx]= find(theta_minus_bounds==minvalue);

    max_r_this_theta=max(Rs(r_bounds(indx)));

    if(r_anchors(f)>max_r_this_theta)
        r_anchors(f)=max_r_this_theta;
    end

    
end



end
