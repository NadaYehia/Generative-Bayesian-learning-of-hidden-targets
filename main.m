
clear all
clc
close all

% setup the environment: the arena size, number of targets,target
% locations, target sizes

targets_xy=[-250 290;10 380; 295 290;-120 335;152 335; ]; % from the mouse data 
targets_sizes=[75 75; 75 75; 75 75; 75 75; 75 75];  % target sizes in x&y: nx2
arena_size=[-375 375 0 750 ];
arena.x=[arena_size(1) arena_size(2) arena_size(2) arena_size(1)];
arena.y=[arena_size(3) arena_size(3) arena_size(4) arena_size(4)];
env=environment;
env.intercept="anypt";
env.blocks=[100 100 100 100 100]; % number of trials per target: 1xn
env.targets_centers=targets_xy; 
env.targets_dimensions= targets_sizes; 
env.arena_dimensions= arena_size;  % arena size 1x4
targets=env.setup_targets_coord; % outputs nx2 struct: each row is a target
                                 % col1 are the x coords and col2 are the 
                                 % ycoords of the target corners

sigma_ridge=7.5; %uncertainity in the value of the action parameters executed.
leak_sigma=[1 1]; % leak in the posterior

ags=20;   %number of agents to run
n=100;    %number of samples in speed space and angle space.
max_speed=839;  %maximum speed value in the action space.
min_speed=0;     % minimum speed value in the action space
max_angle=pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=-pi/2;   %minimum angle in the action space (relative to the vertical axis)
speed_step=round((max_speed-min_speed)/n);
angle_step=((max_angle-min_angle)/n);

Rs=linspace(min_speed,max_speed,n);
Ths=linspace(min_angle,max_angle,n);

clearnce=0.2; %loop width in radians 

% sampler='proportional';
sampler='peak_sampler';

% execution noise
% c=[0.033899299095407 ...
%     12.544571007553474]; % minimum noise in the R estimate fitted to all targets 
                           % on mouse 3                                 
c=[0 0]; %no R noise
bestFitDataOffset=c(2);
bestFitMeanToScaleRatio=c(1);

% bestNormalFitAngleScale=pi*(2.189684343307297/180); % minimum noise in the Theta estimate fitted to
                                                      % all targets of mouse 3
bestNormalFitAngleScale=0; % no hd angle noise

ka=0.01;
draw_flg=0;

% planner optimizer parameters
w2_L=1000;
tol_radius=0.03; % +/-30mm
a_entropy=1; %1mm tighter or wider for 1unit change in entropy
target_order=[1];
kmerge=1;
merging_criterion= (kmerge*sigma_ridge)/n; % converting sigma ridge from pixels distance to normalized dist.
percentile_peak_sampler=0;
min_radius_around_home=50;
relative_surprise= 26;
cache_flag=0;
spc=10000;

[prior_global,theta_bounds,r_bounds]= set_control_actions_space(Rs,Ths,env.arena_dimensions,spc);
prior_global=boolean(prior_global);
%home radius to exclude
[prior_exc_home,~,c_home]=semi_circle_around_home(prior_global,min_radius_around_home,Rs,Ths);
%home radius to exclude

prior_global=prior_global&prior_exc_home;
arena_home_mask=prior_global;
naned_arena_home_mask=1-arena_home_mask;
naned_arena_home_mask(find(naned_arena_home_mask))=nan;
arena_home_mask=arena_home_mask+naned_arena_home_mask;
prior_global=arena_home_mask./(nansum(arena_home_mask(:)));

%% SIMULATION starts

tic
for agent=1:ags

t_lst_caching=1;
initial_ancs=10;
dd=sum(env.blocks(target_order));
surprise_flat=zeros(1,dd);
surprise_working=zeros(1,dd);
caching_times_agent=zeros(1,dd);
mass_out_tent=zeros(1,dd);
mass_outside_home=zeros(1,dd);
posteriors_support_per_agent=zeros(1,dd);
r_loop=zeros(1, dd );
om_main=zeros(1, dd );      
target_hit=[]; 
hit_time = ones(1, dd  ); 
anchors_no= zeros(1,dd);

prior=prior_global;
cched_memories=1;
I_initial=my_entropy(prior);
constantt=log(tol_radius)-(a_entropy*I_initial);

r_anchors_struct={};
theta_anchors_struct={};

anchors_no_variance={};
posterior_support_r_var=zeros(1,dd);
posterior_support_omega_var=zeros(1,dd); 
posterior_support_mu_entropy=zeros(1,dd);
posterior_support_omega_entropy=zeros(1,dd);
tol_vec=zeros(1,dd);
Ivec=zeros(1,dd);

%% starting the simulation for a model mouse

for k=1:dd

     if(k==1) 

       l_ind=[];
       ancs=initial_ancs;
       temp_prior=prior;
       temp_prior(isnan(temp_prior(:)))=0;
       action_array=[1:size(prior,1)*size(prior,2)];
       [l_ind]=datasample(action_array,ancs,'Replace',false,'Weights',temp_prior(:) );
       [theta_temp_ind,r_temp_ind]=ind2sub(size(prior),l_ind);
       r_anchors=Rs(r_temp_ind);
       theta_anchors=Ths(theta_temp_ind);
              
       Ivec(k)=my_entropy(prior);   
       tol_vec(k)=tol_radius;
       
       [Gsol, ~, ~]=connect_anchors_tsp([ 0 r_anchors]',[0  theta_anchors]',initial_ancs+1,Rs,Ths);
       [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
       [rs_,thetas_,pos_x,pos_y]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_vec(k));
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
    L1= (L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) );
    
    % 3- compute the relative surprise in the observed outcome under the current posterior versus under a flat prior 
    [cache_flag,surpW,surpF]=surprise(L1,target_hit(k),prior,relative_surprise,arena_home_mask);
    surprise_flat(k)=surpF;
    surprise_working(k)=surpW;
  
   %disable caching by setting the flag always to 0
   %     cache_flag=0;

   if(~cache_flag)
       
         
        % 4- Baye's update of the control actions space given the outcome of a trajectory: reward=0/1
        [posterior]=Bayes_update_for_actions_params(L1,target_hit(k),prior);
        
        % leak in the posterior certanity every time step; implemented
        % via a 2D gaussian blur
        %          new_posterior=myconv2(posterior,leak_sigma);
        %          posterior=new_posterior;
        
        % compute entropy and reduce tolerance radius
        I=my_entropy(posterior);
        Ivec(k+1)=I;
        dI=Ivec(k+1)-Ivec(k);
        tol_vec(k+1)=exp(a_entropy*Ivec(k+1)+constantt)+(1e-04);
        
        % 5- sampler function for the next actions calling either: proportional or peak sampler
        [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Ths,merging_criterion,r_bounds,theta_bounds,...
        Rs(min(c_home)),percentile_peak_sampler,...
        bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale);
        
        % 6- the generative model connecting a smooth trajectory through the
        % anchors samples.
        if(anchors_no(k+1)>1)
            [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
            [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
            [rs_new,thetas_new,pos_xnew,pos_ynew]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_vec(k+1));
        else
            r_anchors=[0, r_anchors, 0];
            theta_anchors=[0, theta_anchors, 0];
            [rs_new,thetas_new,pos_xnew,pos_ynew]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_vec(k+1));
        end
        prior=posterior;

 
   else
        % 4'- if the current outcome is to surprising, 
        % the environment might have changed and the agent reset its' prior to a flat prior. 
        
        caching_times_agent(k)=1;
        active_prior=prior_global;

        I=my_entropy(active_prior);
        Ivec(k+1)=I;
        dI=Ivec(k+1)-Ivec(k);
        tol_vec(k+1)=exp(a_entropy*Ivec(k+1)+constantt)+(1e-04);
        
        % 5'- sampler function for the next actions calling either: proportional or peak sampler
        [r_anchors,theta_anchors,anchors_no(k+1)]= Sampling_next_actions(active_prior,sampler,initial_ancs,Rs,Ths,merging_criterion,r_bounds,theta_bounds,...
        Rs(min(c_home)),percentile_peak_sampler,...
        bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale);
        % 6- the generative model connecting a smooth trajectory through the
        % anchors samples.
        
        if(anchors_no(k+1)>1)
            [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no(k+1)+1,Rs,Ths);       
            [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
            [rs_new,thetas_new,pos_xnew,pos_ynew]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_vec(k+1));
            
        else
            r_anchors=[0 r_anchors 0];
            theta_anchors=[0, theta_anchors, 0];
            [rs_new,thetas_new,pos_xnew,pos_ynew]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_vec(k+1));
        end
        prior=active_prior;

   end
 
   %% draw the arena, current trajectory, next sampled actions
   if(draw_flg==1)
       figure(2);
       h2=imagesc((Ths),Rs,(prior'));
       set(h2, 'AlphaData', ~isnan(h2.CData))
       set(gca,'YDir','normal');
       set(gca,'XDir','reverse');
       xticks([-pi/2 -pi/3 -pi/4 -pi/8 0 pi/8 pi/4 pi/3 pi/2])
       xticklabels({'-\pi/2','-\pi/3','-\pi/4','-\pi/8','0','\pi/8','\pi/4','\pi/3','\pi/2'})
       set(gca,'TickDir','out')
       hold on, scatter(theta_anchors,r_anchors,30,'r','filled');
%        hold on, scatter(Os(r_b),Rs(c_b),15,'m');
       hold off;
       colorbar;
       f=figure(1);    
       f.Position=[100 100 200 200];
       patch(arena.x,arena.y,[1 1 1]); hold on;
       patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);
       patch(obstacle_.x,obstacle_.y,[0.5 0.5 0.5]);
       hold on, plot(pos_x',pos_y','k','LineWidth',1.5);
       hold on,scatter(pos_x,pos_y,5,[1:numel(pos_y)],'filled');
       set(gca,'XTick',[]);
       set(gca,'YTick',[]);
       
       clf;
       
   end
  %% draw the arena, current trajectory, next sampled actions

  pos_x=[]; pos_y=[];
  % replace the new vectors with the next trajectory xy points
  pos_x=pos_xnew;
  pos_y=pos_ynew;
  % set current actions to the new ones
  rs_=rs_new;
  thetas_=thetas_new;

end
 
r_pop_avg(agent,:) = r_loop;
hd_pop_avg(agent,:)=om_main;
corr_pop_avg(agent,:) = target_hit;
anchors_no_pop(agent,:)=anchors_no;
r_anchors_pop{agent}=r_anchors_struct;
om_anchors_pop{agent}=theta_anchors_struct;
tol_radii(agent,:)=tol_vec;
I_agents(agent,:)=Ivec;
caching_times(agent,:)=caching_times_agent;
posteriors_support(agent,:)=posteriors_support_per_agent;
surprise_working_agent(agent,:)=surprise_working;
surprise_flat_agent(agent,:)=surprise_flat;

end

toc
