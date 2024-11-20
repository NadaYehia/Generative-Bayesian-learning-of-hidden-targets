function [posterior]=Bayes_update_for_actions_params...
    (L1,target_hit,mus_curr,omegas_curr,sigma_ridge,As,Os,prior,wrkrs,arena_home_mask)


if (target_hit==1)
        
     posterior=Bayes_estimate(L1,1,prior);
     posterior=posterior./( nansum(posterior(:)) );
     
       
         
else
         

         posterior=Bayes_estimate(L1,0,prior);
         posterior=posterior./( nansum(posterior(:)) );
        
          
end
  

end