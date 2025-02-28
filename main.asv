% Clear the workspace, command window, and close all figures to start fresh.
clear; clc; close all

%% Target locations and sizes
% Define the (x, y) coordinates of the target locations.
targets_xy=[0 380;-250 290; 250 290;-152 335;152 335; ];  

% Define the width and height of each target.
targets_sizes=[75 75; 75 75; 75 75; 75 75; 75 75];  

% Define the order in which targets will be visited.
target_order=[1 2 3 4 5];

% Define the arena boundaries as [x_min, x_max, y_min, y_max] centered about Home location (x,y).
arena_size=[-375 375 0 750 ];

% Define the number of training trials per target.
training_blcks=[100 100 100 100 100]; 

%% Setup the environment details

% Call the `setup_environment` function to initialize the arena, environment, and targets.
[arena,env,targets]=setup_environment(arena_size,targets_xy,targets_sizes,training_blcks);

%% Simulation parameters:
sigma_ridge=0.075; % Likelihood sigma value in the normalized space.
ags=50;   % number of agents to run
n=100;    % size of the posterior matrix (resolution for action space discretization).
max_speed=839;  %maximum speed value in the action space.
min_speed=0;     % minimum speed value in the action space
max_angle=pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=-pi/2;   %minimum angle in the action space (relative to the vertical axis)
Rs=linspace(min_speed,max_speed,n); % Create a linearly spaced vector of speed values (radial distances).
Ths=linspace(min_angle,max_angle,n); % Create a linearly spaced vector of angle values.
initial_ancs=6; % inital anchors to select.
radius_around_home=50; % maximum distance considered as the Home area.

%% Sampler parameters: 
% % Define the type of sampler to use for selecting actions (e.g., 'peak_sampler').
% sampler='proportional';  
sampler='peak_sampler';

% Scaling factor for the merging criterion.
kmerge=1;

% Distance threshold for merging two anchors in the proportional sampler
merging_criterion= (kmerge*sigma_ridge);  

% Define noise parameters for radial distances and angles (set to zero for no noise).
% . 0.033899299095407 ...
    %12.544571007553474
% c=[0.01 8];    

c=[0 0]; %no R noise

% Offset for radial noise.
radii_noise_offset=c(2);

% Slope for radial noise.
radii_noise_slope=c(1);

% angular_noise=pi*(1/180);

% Define angular noise (set to zero for no noise).
angular_noise=0; %2.189684343307297

% size of local neighborhood for peaks filtration.
roi_size=9; 

%% Visualization parameters

draw_flg=1; % Flag to enable or disable visualization (1 = enable, 0 = disable).

%% Trajectory planner parameters

% Tolerance for radial deviations in the trajectory planner.
tol_radius=0.02;

% scaling factor of the timing function of distance.
rho=10; 

% Time step for trajectory evaluation.
dt=0.01;

% Small epsilon value to avoid numerical issues with
% thetas or phi0_0 reaching the upper and lower bounds in the 
% optimization problem.
eps=1e-2; 

% Maximum number of optimization trials if the initial attempt fails.
max_opti_trials=10; 

% weights for path length and smoothness in the optimization objective 
w1=1; w2=1000;
%% Surprise parameters

% Threshold for detecting surprising outcomes (used for resetting the prior).
working_mem_surprise=0.95; 

% Flag to reset the current prior to a flat one if the outcome is surprising enough.
reset=0; 

%% Set up the posterior space: P(R,theta)

% Call the `setup_posterior_space` function to initialize the posterior space.
[prior_flat,theta_bounds,r_bounds,r_after_home,arena_home_mask] = setup_posterior_space(Rs,Ths,env,radius_around_home);


%% SIMULATION starts

% Delete any existing parallel pool to avoid conflicts.
delete(gcp('nocreate'))
% Start a new parallel pool with 15 workers for parallel processing.
parpool(15);

% Start simulation timer
tic

% Simulate training on all targets for K agents.
for agent=1:ags

    %% Initialize vectors to store results and metrics.

    % Total number of trials based on the target order and training blocks.
    dd=sum(env.blocks(target_order));

    % Array to store surprise values relative to the flat prior.
    surprise_flat=zeros(1,dd);

    % Array to store surprise values relative to the current posterior.
    surprise_working=zeros(1,dd);

    % Array to store times when the prior is reset due to surprising outcomes.
    reset_times=zeros(1,dd);

    % Array to store the mean speed (radius) for each trial.
    r_loop=zeros(1, dd );

    % Array to store the mean heading (angle) for each trial.
    theta_main=zeros(1, dd );   

   % Array to store information about target hits (initially empty).
    target_hit=[]; 

    % Array to store the time stamps where the agent hit the target in each trial.
    hit_time = ones(1, dd  ); 

    % Array to store the number of anchors used in each trial.
    anchors_no= zeros(1,dd);

    % Cell array to store the radial distances of anchors for each trial.
    r_anchors_struct={};

    % Cell array to store the angles of anchors for each trial.
    theta_anchors_struct={};

    
    % Initialize the prior with a flat prior (uniform distribution):
    prior=prior_flat;

    %% Training trials loops:
    for k=1:dd
    
        % For the first trial, select N (initial anchors) at random from
        % the flat prior.
         if(k==1) 
             
           % Set NaN values in the prior to 0 before sampling:
           temp_prior=prior;
           temp_prior(isnan(temp_prior(:)))=0;

           % Create an array of indices for all possible actions:
           action_array=1:size(prior,1)*size(prior,2);

           % Sample initial anchors using weighted random sampling:
           [l_ind]=datasample(action_array,initial_ancs,'Replace',false,'Weights',temp_prior(:) );
           
           % Convert linear indices to row and column indices for the sampled anchors
           [theta_temp_ind,r_temp_ind]=ind2sub(size(prior),l_ind);

           % Extract the radial distances and angles for the sampled anchors:
           r_anchors=Rs(r_temp_ind);
           theta_anchors=Ths(theta_temp_ind);

           %% Fi- Trajectory planning block:
           
           % Use the Travelling Salesman Problem (TSP) algorithm to find the optimal order of anchors:
           [Gsol, ~, ~]=connect_anchors_tsp([ 0 r_anchors]',[0  theta_anchors]',initial_ancs+1,Rs,Ths);

           % Reorder the sampled anchors based on the TSP solution:
           [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);

           % Plan the trajectory between consecutive pairs of anchors:
             [rs_,thetas_,pos_x,pos_y,~,trials_to_optimize]= trajectory_planner_opt_pl_and_smoothness(r_anchors,theta_anchors,env,Ths,Rs,rho,tol_radius,r_after_home,w1,w2,dt,eps,max_opti_trials);

           % Store the number of anchors used in this trial:
           anchors_no(k)=initial_ancs;
    
           % Calculate and store the mean heading and speed for plotting:
           midT=round(numel(rs_)/2);  % Midpoint of the trajectory.
           theta_main(k)=(thetas_(midT)); % Mean heading (angle) at the midpoint.
           r_loop(k)=(rs_(midT));     % Mean speed (radius) at the midpoint.

           % Store the anchors for this trial:
           r_anchors_struct{k}=(r_anchors); 
           theta_anchors_struct{k}=(theta_anchors);
    
         else

            % For subsequent trials, update the mean heading and speed for plotting:
            midT=round(numel(rs_)/2); % Midpoint of the trajectory.
            theta_main(k)=(thetas_(midT)); % Mean heading (angle) at the midpoint.
            r_loop(k)=(rs_(midT));     % Mean speed (radius) at the midpoint. 

            % Store the anchors for this trial:
            r_anchors_struct{k}=(r_anchors);
            theta_anchors_struct{k}=(theta_anchors);
    
         end
    
        %% A- simulate the run and observe outcome

        % Determine the target number for the current trial based on the cumulative sum of training blocks.
        target_num= min(find(k<=cumsum(env.blocks)));  

        % Simulate the run and observe whether the target was hit and the time taken to hit it
        [target_hit(k),hit_time(k)]=simulate_a_run(pos_x,pos_y,target_num,env,hit_time(k));
        
        %% B- compute the likelihood of the target being at any point (R,Theta)
        % given the trajectory and its observed outcome 

        % Compute the likelihood of the target location given the trajectory, observation  and `sigma_ridge`.
        L1=Likelihood(rs_,thetas_,sigma_ridge,Rs,Ths);

        % Mask the computed likelihood with the arena home mask to retain only valid locations:
        % - Outside the home region.
        % - Inside the arena boundaries.
        L1=L1.*arena_home_mask;
        
        % Normalize the likelihood from 0 to 1 for easier interpretation and comparison.
        L1= 0+ ((L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) ))*1;
        
        %% C- compute the relative surprise in the observed outcome under the current posterior versus under a flat prior 
        % Compute the surprise values:
        % - `surpF`: Surprise relative to the flat prior.
        % - `surpW`: Surprise relative to the current posterior.
        % - `reset`: Flag indicating whether to reset the prior due to high surprise.
        [reset,surpW,surpF]=surprise(L1,target_hit(k),prior,working_mem_surprise,arena_home_mask);
        
        % Store the surprise values for this run for visualization
        surprise_flat(k)=surpF; % surprise based on the flat prior
        surprise_working(k)=surpW; % surprise based on the current posterior
    
       if(~reset)
           
            %% D- Baye's update of the control actions space 
            % given the likelihood of the current outcome and the current posterior.

            % Update the posterior using Bayes' rule.
            [posterior]=Bayes_update_for_actions_params(L1,target_hit(k),prior);
            
            % Add a small value to the posterior to avoid numerical instability
            % from repeated multiplications (e.g., probabilities going to 0)
            epsilon = 1e-16;
            posterior = posterior+epsilon;
            posterior = posterior ./ nansum(posterior(:));  % Renormalize the posterior.
            
            %% E- sampler function for the next actions 
            % Call either: proportional or peak sampler to select next
            % actions.

            % Sample the next set of anchors based on the updated posterior.
            [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Ths,merging_criterion,theta_bounds,...
            r_bounds,r_after_home,radii_noise_offset,radii_noise_slope,angular_noise,roi_size);
            
            %% F- Trajectory planning (same as the module in Fi above).
            if(anchors_no(k+1)>1)
                % If there are multiple anchors, use the TSP algorithm to find the optimal order.
                [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
                [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
                [rs_new,thetas_new,pos_xnew,pos_ynew,~,trials_to_optimize]= trajectory_planner_opt_pl_and_smoothness(r_anchors,theta_anchors,env,Ths,Rs,rho,tol_radius,r_after_home,w1,w2,dt,eps,max_opti_trials);

            else
                % If there is only one anchor, append home anchors and plan the trajectory.
                r_anchors=[0, r_anchors, 0];
                theta_anchors=[0, theta_anchors, 0];
                [rs_new,thetas_new,pos_xnew,pos_ynew,~,trials_to_optimize]= trajectory_planner_opt_pl_and_smoothness(r_anchors,theta_anchors,env,Ths,Rs,rho,tol_radius,r_after_home,w1,w2,dt,eps,max_opti_trials);
            end
            
            % Update the prior for the next trial:
            prior=posterior;
           
       else
            % If the current outcome is too surprising, 
            % reset the prior to a flat prior. 

            % Mark this trial as a reset event.
            reset_times(k)=1;

            % Reset the prior to the flat prior.
            prior=prior_flat;
    
            %% E- sampler function for the next actions 
            % Call either: proportional or peak sampler to select next
            % actions.

            % Sample the next set of anchors based on the flat prior.
            [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(prior,sampler,initial_ancs,Rs,Ths,merging_criterion,theta_bounds,...
            r_bounds,r_after_home,radii_noise_offset,radii_noise_slope,angular_noise,roi_size);
    
            %% F the generative model connecting a smooth trajectory through the
            % anchors samples.

            if(anchors_no(k+1)>1)
                % If there are multiple anchors, use the TSP algorithm to find the optimal order.
                [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
                [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
                [rs_new,thetas_new,pos_xnew,pos_ynew,~,trials_to_optimize]= trajectory_planner_opt_pl_and_smoothness(r_anchors,theta_anchors,env,Ths,Rs,rho,tol_radius,r_after_home,w1,w2,dt,eps,max_opti_trials);

            else
                % If there is only one anchor, append home anchors and plan the trajectory.
                r_anchors=[0 r_anchors 0];
                theta_anchors=[0, theta_anchors, 0];
                [rs_new,thetas_new,pos_xnew,pos_ynew,~,trials_to_optimize]= trajectory_planner_opt_pl_and_smoothness(r_anchors,theta_anchors,env,Ths,Rs,rho,tol_radius,r_after_home,w1,w2,dt,eps,max_opti_trials);

            end
    
       end
     
       % Visualization:
       if(draw_flg==1)
           % Draw the current trial's details (target, trajectory, anchors, etc.).
           draw_trial(Ths,Rs,prior,theta_anchors,r_anchors,arena,targets,target_num,pos_x,pos_y);

           % Clear the figure for the next trial.
           clf;
       end
    
     % Update the trajectory and anchors for the next run:
      pos_x=pos_xnew;  % Update the x-coordinates of the trajectory.
      pos_y=pos_ynew;  % Update the y-coordinates of the trajectory.
      rs_=rs_new;      % Update the radial distances of the trajectory.
      thetas_=thetas_new;  % Update the angles of the trajectory.
    
    end
    
    % Store loops radii, heading angles and accuracies traces from all training trials 
    % for every model mouse agent
    r_pop_avg(agent,:) = r_loop;
    hd_pop_avg(agent,:)=theta_main;
    corr_pop_avg(agent,:) = target_hit;
    anchors_no_pop(agent,:)=anchors_no;
    r_anchors_pop{agent}=r_anchors_struct;
    theta_anchors_pop{agent}=theta_anchors_struct;
    caching_times(agent,:)=reset_times;
    surprise_working_agent(agent,:)=surprise_working;
    surprise_flat_agent(agent,:)=surprise_flat;

end

% Stop simulation timer
toc
