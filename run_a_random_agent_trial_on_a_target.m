function [mu_spd,om_main,target_hit,anchors_no,mu_anchors_variance,...
         omega_anchors_variance,anchors_no_variance,posterior_support_mu_var,posterior_support_omega_var,...
          posterior_support_mu_entropy,posterior_support_omega_entropy]=    run_a_random_agent_trial_on_a_target (blocks,arena,target,target2,target_num,r_bounds,c_bounds,prior,initial_ancs,As,Os,sampler,...
                                               merging_criterion,sigma_ridge,speed_step,draw_flg)

dd=sum(blocks(target_num));

mu_spd=zeros(1, dd );
om_main=zeros(1, dd );      
target_hit=zeros(1, dd ); 
hit_time = ones(1, dd  ); 
anchors_no= zeros(1,dd);
mu_anchors_variance={};
omega_anchors_variance={};
anchors_no_variance={};
posterior_support_mu_var=zeros(1,dd);
posterior_support_omega_var=zeros(1,dd);
 
posterior_support_mu_entropy=zeros(1,dd);
posterior_support_omega_entropy=zeros(1,dd);

for k=1:dd
             
     % for first run, pick n action anchors randomly from the prior, connect
     % them smoothly in xy space and find all the actions along this smooth
     % trajectory

     if(k==1) 

       % avoiding boundary actions, to prevent choosing a false peak when 
       % using the peak sampler: which find the maximum action in a 3x3
       % local region, the boundary pixels (e.g. the first column) will be 
       % compared with the last column and become a *false peak*!
       
       lin_idx_bounds=sub2ind(size(prior),r_bounds,c_bounds);
       lrw=size(prior,1);
       % add the indices for the first row, last row, and first column
       ind_first_row= sub2ind(size(prior),repmat(1,size(prior,2),1), (1:size(prior,2))');
       ind_last_row= sub2ind(size(prior),repmat(lrw,size(prior,2),1),(1:size(prior,2))');
       lin_idx_bounds=[lin_idx_bounds, [1:size(prior,1)], ind_first_row', ind_last_row'];
       
       l_ind=[];
       ancs=initial_ancs;
    
       % sample till all the first n anchors are non-boundary
       % actions
       while(numel(l_ind)~=initial_ancs)
        
        action_array=[1:size(prior,1)*size(prior,2)];
        [temp]=datasample(action_array,ancs,'Replace',false,'Weights',(prior(:)));
    
        not_peaks=find(ismember(temp,lin_idx_bounds));
        temp(not_peaks)=[];
        l_ind=[l_ind,temp];
        ancs=initial_ancs-numel(l_ind);
    
       end
    
       [omega_temp_ind,mu_temp_ind]=ind2sub(size(prior),l_ind);
    
       mu_anchors=As(mu_temp_ind);
       omega_anchors=Os(omega_temp_ind);
                                         
       [~,min_speed_anchor]=min(mu_anchors);
       initial_hd=omega_anchors(min_speed_anchor);
    
       % apply tsp on the anchors points
       Gsol=connect_anchors_tsp([0 mu_anchors]',[ initial_hd omega_anchors]',initial_ancs+1,As,Os);
         
       % reorder the anchors according to the TSP list order
       [mu_anchors,omega_anchors]=reorder_actions_anchors([0 mu_anchors],[ initial_hd omega_anchors],Gsol);
    
       % interpolate between the ordered anchors actions, retrieve the full
       % x,y list of points AND the model loops actions params peaking at these points.
       [mus_,omegas_,pos_x,pos_y]= connect_actions_with_smooth_trajectory(mu_anchors,omega_anchors,sigma_ridge,speed_step);
    
       % keep track of mean heading and speeds
       om_main(k)=mean(omegas_);
       mu_spd(k)=mean(mus_);      

     else
        % keep track of mean heading and speeds
        om_main(k)=mean(omegas_);
        mu_spd(k)=mean(mus_);   
       
     end

   [target_hit(k),hit_time(k)]=simulate_a_run(pos_x,pos_y,arena,target_num,target,target2,target_hit(k),hit_time(k));

   [posterior]=Bayes_update_for_actions_params(target_hit(k),mus_,omegas_,sigma_ridge,As,Os,prior);
   
   [mu_anchors,omega_anchors,anchors_no(k)]= Sampling_next_actions(posterior,sampler,initial_ancs,As,Os,merging_criterion,r_bounds,c_bounds);
   
    [mus_next,omegas_next,pos_xnext,pos_ynext]= Generative_model_connecting_anchors_with_smooth_traj(mu_anchors,omega_anchors,...
                                                 anchors_no(k),As,Os,sigma_ridge,speed_step);
   
   prior=posterior;
   
   %% compute variability of posterior Vs variability in anchors

     [lind]= datasample(action_array,100,'Replace',false,'Weights',(posterior(:)));
     [omega_ind,mu_ind]=ind2sub(size(posterior),lind);
    
     posterior_support_mu_var(k)= var(As(mu_ind));
     posterior_support_omega_var(k)=var(Os(omega_ind));
    
     posterior_support_omega_entropy(k)=my_entropy(posterior);
     posterior_support_mu_entropy(k)=my_entropy(posterior);


     mu_anchors_variance{k}=(mu_anchors);
     omega_anchors_variance{k}=(omega_anchors);
     anchors_no_variance{k}=anchors_no(k);

   %% draw the arena, current trajectory, next sampled actions
   if(draw_flg==1)
       figure(2);
       imagesc(As,Os,prior);
      
       hold on, scatter(mu_anchors,omega_anchors,30,'r','filled');
       hold off;
       figure(1);       
       hold on, plot(pos_x,pos_y,'LineWidth',0.5,'Color',[1 0.1 0.8 0.5]);
       hold on,scatter(pos_x,pos_y,10,[1:numel(pos_y)],'filled');
      
       clf;
       patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
       patch(target.x,target.y,[0.85 0.9 0.9]);
       patch(target2.x,target2.y,[0.9 0.9 0.85]);
   end
 

  % empty the current trajectory posx and posy variables
   pos_x=[]; pos_y=[];
  % replace the new vectors with the next trajectory xy points
  pos_x=pos_xnext;
  pos_y=pos_ynext;

 % set current actions to the ones sampled from line 71
  mus_=mus_next;
  omegas_=omegas_next;

end
% save variables for further analytics


end