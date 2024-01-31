function posterior= Bayes_estimate(poss_actions,r,prior)
    
L=[];

    if(r==0)
        L=(1-poss_actions);
    
    else
        L=poss_actions;
    
    end
    
    


posterior=L.*prior;

end