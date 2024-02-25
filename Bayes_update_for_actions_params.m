function [posterior]=Bayes_update_for_actions_params...
    (target_hit,mus_curr,omegas_curr,sigma_ridge,As,Os,prior,wrkrs)

% the agent assumes a likelihood of 1 at the executed action params that
% has a smooth fall off controlled by the parameter *sigma_ridge*

L1=place_field_map_for_likelihood(mus_curr,omegas_curr,sigma_ridge,As,Os,wrkrs);

L1= (L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) );
 

if (target_hit==1)
        
     posterior=Bayes_estimate(L1,1,prior);
     posterior=posterior./( sum(sum(posterior)) );
     
       
         
else
         

         posterior=Bayes_estimate(L1,0,prior);
         posterior=posterior./( sum(sum(posterior)) );
        
          
end
  

end