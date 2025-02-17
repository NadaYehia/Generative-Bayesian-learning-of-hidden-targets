%         decayed_prior=1e-40-((1e-40-prior).*exp(-t/1000));
%         prior=decayed_prior./nansum(decayed_prior(:));
%         t=t+1;
%         K=[pt pt pt;pt (1-8*pt) pt;pt pt pt];
% 
%         convolved_prior=nanconv2(prior,K);
%         convolved_prior=convolved_prior.*arena_home_mask;
%         prior=convolved_prior./nansum(convolved_prior(:));

theta_bound_1=[atan2(arena(3),arena(2)),...
    atan2(arena(4),arena(2))];

theta_bound_3=[atan2(arena(4),arena(1)),...
    atan2(arena(3),arena(1))];

theta_bound_2=[atan2(arena(4),arena(2)),...
    atan2(arena(4),arena(1))];



%shift the anchors angles by pi/2 to the same reference frame of the
    %arena wall functions (i.e.[0->pi].
    theta_n=theta_anchors(f)+(pi/2);

    r_n=r_anchors(f);
   
    if theta_n>theta_bound_1(1) && theta_n<=theta_bound_1(2) 
             
             % compare the anchor's radius with the radius evaluated at
             % this angle by the (r,theta) arena wall function

             if (r_n> (arena(2)/cos(theta_n))) %if noisy anchor radius is greater than
                                               % maximum allowed radius,
                                               % set it to the maximum
                                               % radius.
                 r_anchors(f)=(arena(2)/cos(theta_n));
             end
    
    elseif theta_n>theta_bound_3(1) && theta_n<=theta_bound_3(2)  
        
             % compare the anchor's radius with the radius evaluated at
             % this angle by the (r,theta) arena wall function

             if( r_n> (arena(1)/cos(theta_n))) %if noisy anchor radius is greater than
                                               % maximum allowed radius,
                                               % set it to the maximum
                                               % radius.
                 r_anchors(f)=(arena(1)/cos(theta_n));
             end

    elseif  theta_n>theta_bound_2(1) && theta_n<=theta_bound_2(2)

             % compare the anchor's radius with the radius evaluated at
             % this angle by the (r,theta) arena wall function

             if(r_n> (arena(4)/sin(theta_n)))  %if noisy anchor radius is greater than
                                               % maximum allowed radius,
                                               % set it to the maximum
                                               % radius.
                 r_anchors(f)=(arena(4)/sin(theta_n));
             end
    end