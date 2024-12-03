% %% 
% Gsol=connect_anchors_tsp([0 As(250) As(250)]',[Os(250) Os(250) Os(251)]',3,As,Os);
% 
% [mu_anchors_test,omega_anchors_test]=reorder_actions_anchors([0 As(250) As(250)],[Os(250) Os(250) Os(251)],Gsol);

%%
for k=1:10000

        initial_ancs=10;
        dd=sum(env.blocks(target_num));
        [prior,r_bounds,c_bounds]= set_control_actions_space(As,Os,env.arena_dimensions,clearnce,T);
       
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
    
      %% the generative model connecting anchors with a smooth trajectory 
       Gsol=connect_anchors_tsp([0 mu_anchors]',[ initial_hd omega_anchors]',initial_ancs+1,As,Os);
       [mu_anchors,omega_anchors]=reorder_actions_anchors([0 mu_anchors],[ initial_hd omega_anchors],Gsol);
       [mus_,omegas_,pos_x,pos_y]= connect_actions_with_smooth_trajectory(mu_anchors,omega_anchors,sigma_ridge,speed_step,env,clearnce,Drift,T,Os,As,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac);
       anchors_no(k)=initial_ancs;
       % keep track of mean heading and speeds
       midT=numel(mus_)/2;
       om_main(k)=(omegas_(midT));
       mu_spd(k)=(mus_(midT));
       
       mu_anchors_struct{k}=(mu_anchors);
       omega_anchors_struct{k}=(omega_anchors);
end

%% can anchors themselves (average) value be consistently different from 
%% the om main measured as the omega from the trajectory at Tmidway?
