function [active_prior,cached_tensors,maps_exh]=caching_routine(prior,cached_tensors,...
    maps_exh,n_maps,outcome,current_actions,Rs,Os,env,min_radius_around_home)


% if no cached tensors before then cache,
if(size(cached_tensors,3)==1 && all(cached_tensors(:)==0))

    cached_tensors(:,:,1)=prior;

elseif(maps_exh==0)% or if it is a prior coming from a flat prior, cache,

    cached_tensors(:,:,end+1)=prior;

end

% compute surprise on (n_maps) cached_tensors
% pick one with minimum surprise and 

if(size(cached_tensors,3)>1)
    for p=size(cached_tensors,3):-1:size(cached_tensors,3)-(n_maps)+1
    
        for ac=2:size(current_actions,1)-1
    
            [~,omega_acs_idx]= min(abs(Os -current_actions(ac,1)) );
            [~,r_acs_idx]=min( abs(Rs-current_actions(ac,2)) );
            pr_post(ac)=cached_tensors(omega_acs_idx,r_acs_idx,p);
    
        end
        pr_post(1)=[];
        temp_p=max(pr_post);
    
        if (outcome==1)
            surprises(p)=-log2(temp_p);
        
        else
            surprises(p)=-log2(1-temp_p);
        
        end
        
    
    end
end

% set active prior to be a flat prior if (n_maps) to try=maps_exhausted
% the n_maps set in the main script =2
% set prior to flat when the current posterior before the cache and the one before it
% don't work on the new environment, (n_maps) 2 maps are tried 

if( maps_exh==(n_maps-1) || size(cached_tensors,3)==1 )

    
    [flat_prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,env.arena_dimensions);
    [prior_exc_home,r_home,c_home]=semi_circle_around_home(flat_prior,min_radius_around_home,Rs,Os);
    flat_prior=flat_prior.*prior_exc_home;
    flat_prior=flat_prior./(sum(sum(flat_prior)));
    active_prior=flat_prior;
    maps_exh=0;


else 
    % choose the cached map with the minimum surprise, for n_maps=2
    % it is the current one and the one before it;  
    [~,min_S]=min(surprises(p:end));
    active_prior=cached_tensors(:,:,(min_S-1)+p);
    maps_exh=maps_exh+1;
 
end


      
end
