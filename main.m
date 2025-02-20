
clear; clc; close all

% Target locations and sizes
targets_xy=[0 380;-250 290; 250 290;-152 335;152 335; ];  
targets_sizes=[75 75; 75 75; 75 75; 75 75; 75 75];  % target sizes in w&h
target_order=[1 2 3 4 5];

arena_size=[-375 375 0 750 ]; %arena coordinates [x1 x2 y1 y2]
training_blcks=[100 100 100 100 100]; %number of trials per target

% Setup the environment details
[arena,env,targets]=setup_environment(arena_size,targets_xy,targets_sizes,training_blcks);

% Simulation parameters:
sigma_ridge=0.075; % Likelihood sigma value in the normalized space.
ags=10;   % number of agents to run
n=100;    % size of the posterior matrix.
max_speed=839;  %maximum speed value in the action space.
min_speed=0;     % minimum speed value in the action space
max_angle=pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=-pi/2;   %minimum angle in the action space (relative to the vertical axis)
Rs=linspace(min_speed,max_speed,n);
Ths=linspace(min_angle,max_angle,n);
initial_ancs=6; % inital anchors to select.
radius_around_home=50; % distance considered as Home up till this value.

% Sampler parameters: 
% sampler type
% sampler='proportional';  
sampler='peak_sampler';

kmerge=1;
merging_criterion= (kmerge*sigma_ridge); % distance threshold for merging 2 anchors 
                                         % in the proportional sampler

% Sampler noise estimates from the mouse data. 0.033899299095407 ...
    %12.544571007553474
% c=[0.01 8];                               
c=[0 0]; %no R noise
radii_noise_offset=c(2);
radii_noise_slope=c(1);
% angular_noise=pi*(1/180);
angular_noise=0; % no hd angle noise,2.189684343307297
min_local_diff=1e-6;
pr_thresh=0.01;
roi_size=7;


% Visualization flag
draw_flg=0;

% Trajectory planner parameters
w1_L=1; tol_radius=0.01; ka=10;

% Surprise threshold
working_mem_surprise=1;
reset=0; % a flag to reset current prior to a flat one if the outcome is surprising enough.

% Set up the posterior space: P(R,theta)
[prior_flat,theta_bounds,r_bounds,r_after_home,arena_home_mask] = setup_posterior_space(Rs,Ths,env,radius_around_home);

% SIMULATION starts
delete(gcp('nocreate'))
parpool(15);

tic
parfor agent=1:ags

    dd=sum(env.blocks(target_order));
    surprise_flat=zeros(1,dd);
    surprise_working=zeros(1,dd);
    reset_times=zeros(1,dd);
    r_loop=zeros(1, dd );
    om_main=zeros(1, dd );      
    target_hit=[]; 
    hit_time = ones(1, dd  ); 
    anchors_no= zeros(1,dd);
    prior=prior_flat;
    r_anchors_struct={};
    theta_anchors_struct={};
    
    % training trials:
    for k=1:dd
    
         if(k==1) 
    
           temp_prior=prior;
           temp_prior(isnan(temp_prior(:)))=0;
           action_array=1:size(prior,1)*size(prior,2);
           [l_ind]=datasample(action_array,initial_ancs,'Replace',false,'Weights',temp_prior(:) );
           [theta_temp_ind,r_temp_ind]=ind2sub(size(prior),l_ind);
           r_anchors=Rs(r_temp_ind);
           theta_anchors=Ths(theta_temp_ind);
           
           % planning a trajectory through the first set of anchors using trajectory planner and TSP 
           % to order these anchors:
           [Gsol, ~, ~]=connect_anchors_tsp([ 0 r_anchors]',[0  theta_anchors]',initial_ancs+1,Rs,Ths);
           [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
           [rs_,thetas_,pos_x,pos_y,~]= trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,ka,w1_L,tol_radius,r_after_home);
           anchors_no(k)=initial_ancs;
    
           % keep track of mean heading and speeds
           midT=round(numel(rs_)/2);
           om_main(k)=(thetas_(midT));
           r_loop(k)=(rs_(midT));
           r_anchors_struct{k}=(r_anchors);
           theta_anchors_struct{k}=(theta_anchors);
    
         else
            % keep track of mean heading and speeds
            midT=round(numel(rs_)/2);
            om_main(k)=(thetas_(midT));
            r_loop(k)=(rs_(midT));   
            r_anchors_struct{k}=(r_anchors);
            theta_anchors_struct{k}=(theta_anchors);
    
         end
    
        % 1- simulate the run and observe outcome
        target_num= min(find(k<=cumsum(env.blocks)));  
        [target_hit(k),hit_time(k)]=simulate_a_run(pos_x,pos_y,target_num,env,hit_time(k));
        
        % 2- compute the likelihood of the target being at any point (R,Theta)
        % given the trajectory and its observed outcome 
        L1=Likelihood(rs_,thetas_,sigma_ridge,Rs,Ths);
        L1=L1.*arena_home_mask;
        L1= 0+ ((L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) ))*1;
        
        % 3- compute the relative surprise in the observed outcome under the current posterior versus under a flat prior 
        [reset,surpW,surpF]=surprise(L1,target_hit(k),prior,working_mem_surprise,arena_home_mask);
        surprise_flat(k)=surpF;
        surprise_working(k)=surpW;
        
    
       if(~reset)
           
             
            % 4- Baye's update of the control actions space given the outcome of a trajectory: reward=0/1
            [posterior]=Bayes_update_for_actions_params(L1,target_hit(k),prior);
            
            % add a small value to the posterior to avoid its entries going to 0 from 
            % repeated multiplications
            epsilon = 1e-16;
            posterior = posterior+epsilon;
            posterior = posterior ./ nansum(posterior(:));  % Renormalize
            
            % 5- sampler function for the next actions calling either: proportional or peak sampler
            [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Ths,merging_criterion,theta_bounds,...
            r_bounds,r_after_home,radii_noise_offset,radii_noise_slope,angular_noise,min_local_diff,pr_thresh,roi_size);
            
            % 6- the generative model connecting a smooth trajectory through the
            % anchors samples.
            if(anchors_no(k+1)>1)
                [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
                [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
                [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg]= trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,ka,w1_L,tol_radius,r_after_home);
            else
                r_anchors=[0, r_anchors, 0];
                theta_anchors=[0, theta_anchors, 0];
                [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg]= trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,ka,w1_L,tol_radius,r_after_home);
            end
            
            prior=posterior;
           
       else
            % 4'- if the current outcome is too surprising, 
            % the environment might have changed and the agent reset its' prior to a flat prior. 
            reset_times(k)=1;
            prior=prior_flat;
    
            % 5'- sampler function for the next actions calling either: proportional or peak sampler
            [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(prior,sampler,initial_ancs,Rs,Ths,merging_criterion,theta_bounds,...
            r_bounds,r_after_home,radii_noise_offset,radii_noise_slope,angular_noise,min_local_diff,pr_thresh,roi_size);
    
            % 6- the generative model connecting a smooth trajectory through the
            % anchors samples.
            if(anchors_no(k+1)>1)
                [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
                [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
                [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg]= trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,ka,w1_L,tol_radius,r_after_home);
                
            else
                r_anchors=[0 r_anchors 0];
                theta_anchors=[0, theta_anchors, 0];
                [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg]= trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,ka,w1_L,tol_radius,r_after_home);
            end
    
       end
     
       % draw the arena, current trajectory, next sampled actions
       if(draw_flg==1)
           draw_trial(Ths,Rs,prior,theta_anchors,r_anchors,arena,targets,target_num,pos_x,pos_y);
           clf;
       end
    
      % set next run's trajectories and anchors to the current output from the trajectory planner
      pos_x=pos_xnew;
      pos_y=pos_ynew;
      rs_=rs_new;
      thetas_=thetas_new;
    
    end
    
    % store loops radii, heading angles and accuracies traces per model mouse agent
    r_pop_avg(agent,:) = r_loop;
    hd_pop_avg(agent,:)=om_main;
    corr_pop_avg(agent,:) = target_hit;
    anchors_no_pop(agent,:)=anchors_no;
    r_anchors_pop{agent}=r_anchors_struct;
    om_anchors_pop{agent}=theta_anchors_struct;
    caching_times(agent,:)=reset_times;
    surprise_working_agent(agent,:)=surprise_working;
    surprise_flat_agent(agent,:)=surprise_flat;

end

toc
