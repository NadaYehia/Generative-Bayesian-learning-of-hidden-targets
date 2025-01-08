function [posterior]=Bayes_update_for_actions_params...
    (L1,target_hit,prior)


if (target_hit==1)
        
     posterior=Bayes_estimate(L1,1,prior);
     posterior=posterior./( nansum(posterior(:)) );
     
       
         
else
         

         posterior=Bayes_estimate(L1,0,prior);
         posterior=posterior./( nansum(posterior(:)) );
        
          
end
  

end