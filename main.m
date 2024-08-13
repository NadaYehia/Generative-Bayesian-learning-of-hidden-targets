
clear all
clc
% setup the environment: the arena size, number of target, special objects
% (e.g. obstacles) *to be added*
close all
load('Drift.mat')
global failuree;
failuree=1;

targets_xy=[0 300; -283 100; 283 100; -169 250; 102.5 250]; % -49.5 295, 2Dcenters per target: nx2 : from mouse#3, last days
targets_sizes=[60 60; 60 60; 60 60; 60 60; 60 60];  % target sizes in x&y: nx2

arena_size=[-400 400 0 600];
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


sigma_ridge=20; %uncertainity in the value of the action parameters executed.
ags=50;   %number of agents to run
n=500;    %number of samples in speed space and angle space.
max_speed=1000;  %maximum speed value in the action space.
min_speed=1;     % minimum speed value in the action space
max_angle=pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=-pi/2;   %minimum angle in the action space (relative to the vertical axis)
speed_step=round((max_speed-min_speed)/n);
angle_step=((max_angle-min_angle)/n);
Rs=linspace(min_speed,max_speed,n);
Os=linspace(min_angle,max_angle,n);
clearnce=0.5; %loop width in radians 

sampler='proportional';
% sampler='peak_sampler';

%% execution noise
c=[0.033899299095407 ...
    12.544571007553474]; %minimum execution distance noise for mouse 3
                                 % minimum noise variance and variance to
                                 % mean ratio

% c=[0 0]; %no noise, 0.00496923501318870

bestFitDataOffset=c(2);
bestFitMeanToScaleRatio=c(1);

bestNormalFitAngleScale=pi*(2.189684343307297/180); %minimum angle noise on all targets of mouse 3

% bestNormalFitAngleScale=0; %no hd angle noise

%%
drift_fac=0;
ka=0.01;
draw_flg=0;

% planner optimizer parameters
w2_L=50;
tol_radius=0.03; % +/-30mm
a_entropy=1; %1mm tighter or wider for 1unit change in entropy

%
target_num=3;
target_order=[1 2];
kmerge=5;

merging_criterion= (kmerge*sigma_ridge)/n; % converting sigma ridge from pixels distance to normalized dist.
wrkrs=4;
boundary_wash_out=11;
percentile_peak_sampler=0.6;
min_radius_around_home=50;


%%
[prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,env.arena_dimensions);
%home radius to exclude
[prior_exc_home,r_home,c_home]=semi_circle_around_home(prior,min_radius_around_home,Rs,Os);
%home radius to exclude

lin_idx_bounds_before_conv_1=sub2ind(size(prior),r_bounds,c_bounds);  
lin_idx_bounds_before_conv_2=sub2ind(size(prior),r_home,c_home);
lin_idx_bounds_before_conv=[lin_idx_bounds_before_conv_1, lin_idx_bounds_before_conv_2]; 

vc1=min(c_home);
col_home=repmat(vc1,1,size(prior,1));
ind_least_col_in_mask1= sub2ind(size(prior),[1:size(prior,1)],col_home);


lin_idx_bounds_before_conv=[lin_idx_bounds_before_conv, ind_least_col_in_mask1 ];
locs_boundary=zeros(size(prior));
locs_boundary(lin_idx_bounds_before_conv)=1;
conv_locs_boundary=conv2(locs_boundary,ones(boundary_wash_out),'same');
lin_idx_bounds= find(conv_locs_boundary);
%%

relative_surprise=1000;
cache_flag=0;


tic

% start agents trials
parfor agent=1:ags

t_lst_caching=1;
initial_ancs=10;
dd=sum(env.blocks(target_order));

maps_exhausted=0;
n_maps=2;

caching_times_agent=zeros(1,dd);

r_loop=zeros(1, dd );
om_main=zeros(1, dd );      
target_hit=[]; 
hit_time = ones(1, dd  ); 
anchors_no= zeros(1,dd);
[prior,r_bounds,c_bounds]= set_control_actions_space(Rs,Os,env.arena_dimensions);
%home radius to exclude
[prior_exc_home,r_home,c_home]=semi_circle_around_home(prior,min_radius_around_home,Rs,Os);
%home radius to exclude
prior=prior.*prior_exc_home;
prior=prior./(sum(sum(prior)));

cached_tensors=zeros(size(prior));
cched_memories=1;
I_initial=my_entropy(prior);
constantt=log(tol_radius)-(a_entropy*I_initial);

r_anchors_struct={};
omega_anchors_struct={};

anchors_no_variance={};
posterior_support_r_var=zeros(1,dd);
posterior_support_omega_var=zeros(1,dd); 
posterior_support_mu_entropy=zeros(1,dd);
posterior_support_omega_entropy=zeros(1,dd);
tol_vec=zeros(1,dd);
Ivec=zeros(1,dd);

%% starting the simulation for a model agent
for k=1:dd

     if(k==1) 

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
    
       [omega_temp_ind,r_temp_ind]=ind2sub(size(prior),l_ind);
    
       r_anchors=Rs(r_temp_ind);
       omega_anchors=Os(omega_temp_ind);
              
       Ivec(k)=my_entropy(prior);   
       tol_vec(k)=tol_radius;
       [~,min_radius_anchor]=min(r_anchors);
       initial_hd=omega_anchors(min_radius_anchor);
    
      %% the generative model connecting anchors with a smooth trajectory 
       Gsol=connect_anchors_tsp([0 r_anchors]',[ initial_hd omega_anchors]',initial_ancs+1,Rs,Os);
       [r_anchors,omega_anchors]=reorder_actions_anchors([0 r_anchors],[ initial_hd omega_anchors],Gsol);
       [rs_,omegas_,pos_x,pos_y]= connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac,ka,w2_L,tol_vec(k),failuree);
       anchors_no(k)=initial_ancs;

       % keep track of mean heading and speeds
       midT=round(numel(rs_)/2);

       om_main(k)=(omegas_(midT));
       r_loop(k)=(rs_(midT));
       
       r_anchors_struct{k}=(r_anchors);
       omega_anchors_struct{k}=(omega_anchors);

     else
        % keep track of mean heading and speeds
        midT=round(numel(rs_)/2);

        om_main(k)=(omegas_(midT));
        r_loop(k)=(rs_(midT));   
        r_anchors_struct{k}=(r_anchors);
        omega_anchors_struct{k}=(omega_anchors);

     end
    target_num= min(find(k<=cumsum(env.blocks)));
    
   [target_hit(k),hit_time(k)]=simulate_a_run(pos_x,pos_y,target_num,env,hit_time(k));


%% caching logic, based on posterior and reward rate signal entropies

   [cache_flag]=caching_surprise(target_hit(k),prior,[omegas_',rs_'],Rs,Os,relative_surprise,env,min_radius_around_home);

   if(~cache_flag)
         
        
        %% Baye's update of the control actions space given the outcome of a trajectory: reward=0/1
         [posterior]=Bayes_update_for_actions_params(target_hit(k),rs_,omegas_,sigma_ridge,Rs,Os,prior,wrkrs);

      %% compute entropy and reduce tolerance radius
         I=my_entropy(posterior);
         Ivec(k+1)=I;
         dI=Ivec(k+1)-Ivec(k);
         tol_vec(k+1)=exp(a_entropy*Ivec(k+1)+constantt)+(1e-04);

      %% sampler function for the next actions calling either: proportional or peak sampler
         [r_anchors,omega_anchors,anchors_no(k+1)]= Sampling_next_actions(posterior,sampler,initial_ancs,Rs,Os,merging_criterion,lin_idx_bounds,percentile_peak_sampler);

         [~,min_radius_anchor]=min(r_anchors);
         initial_hd=omega_anchors(min_radius_anchor);
   
      if(anchors_no(k+1)>1)
         
         Gsol=connect_anchors_tsp([0 r_anchors]',[ initial_hd omega_anchors]',anchors_no(k+1)+1,Rs,Os);       
         [r_anchors,omega_anchors]=reorder_actions_anchors([0 r_anchors],[ initial_hd omega_anchors],Gsol);
         [rs_new,omegas_new,pos_xnew,pos_ynew]= connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac,ka,w2_L,tol_vec(k+1),failuree);

      
      else
      
          r_anchors=[0 r_anchors 0];
         omega_anchors=[omega_anchors, omega_anchors, omega_anchors];
         %% the generative model connecting anchors with a smooth trajectory 
         [rs_new,omegas_new,pos_xnew,pos_ynew]= connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac,ka,w2_L,tol_vec(k+1),failuree);
      
      end
         prior=posterior;

   
  
   else
       % you cached your old posterior and started with a new one. 
       caching_times_agent(k)=1;

       [active_prior,cached_tensors,maps_exhausted]=caching_routine(prior,cached_tensors,maps_exhausted,n_maps,...
           target_hit(k),[omegas_',rs_'],Rs,Os,env,min_radius_around_home);
       
       %% compute tolerance from the active prior entropy
         
         I=my_entropy(active_prior);
         Ivec(k+1)=I;
         dI=Ivec(k+1)-Ivec(k);
         tol_vec(k+1)=exp(a_entropy*Ivec(k+1)+constantt)+(1e-04);

       %% sample and plan the new route from the current active posterior.
         [r_anchors,omega_anchors,anchors_no(k+1)]= Sampling_next_actions(active_prior,sampler,initial_ancs,Rs,Os,merging_criterion,lin_idx_bounds,percentile_peak_sampler);

         [~,min_radius_anchor]=min(r_anchors);
         initial_hd=omega_anchors(min_radius_anchor);
   
      if(anchors_no(k+1)>1)
         
         Gsol=connect_anchors_tsp([0 r_anchors]',[ initial_hd omega_anchors]',anchors_no(k+1)+1,Rs,Os);       
         [r_anchors,omega_anchors]=reorder_actions_anchors([0 r_anchors],[ initial_hd omega_anchors],Gsol);
         [rs_new,omegas_new,pos_xnew,pos_ynew]= connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac,ka,w2_L,tol_vec(k+1),failuree);

      
      else
      
         r_anchors=[0 r_anchors 0];
         omega_anchors=[omega_anchors, omega_anchors, omega_anchors];
         %% the generative model connecting anchors with a smooth trajectory 
         [rs_new,omegas_new,pos_xnew,pos_ynew]= connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale,drift_fac,ka,w2_L,tol_vec(k+1),failuree);
      
      end

       % the current flat posterior is the only posterior that will be updated; 
       %i.e. the old caches remain untouched
          prior=active_prior;

   end
    
   %posteriors_per_agent(k,:,:)=posterior;
   %% compute variability of posterior Vs variability in anchors

     [lind]= datasample(action_array,100,'Replace',false,'Weights',(posterior(:)));
     [omega_ind,r_ind]=ind2sub(size(posterior),lind);
    
     posterior_support_r_var(k)= var(Rs(r_ind));
     posterior_support_omega_var(k)=var(Os(omega_ind));
    
     posterior_support_omega_entropy(k)=my_entropy(posterior);
     posterior_support_mu_entropy(k)=my_entropy(posterior);


     anchors_no_variance{k}=anchors_no(k);

     [r_b,c_b]=ind2sub(size(prior),lin_idx_bounds);
   %% draw the arena, current trajectory, next sampled actions
   if(draw_flg==1)
       figure(2);
       imagesc(fliplr(Os),Rs,fliplr(prior'));
       set(gca,'YDir','normal');
       set(gca,'XDir','reverse');
       xticks([-pi/2 -pi/3 -pi/4 -pi/8 0 pi/8 pi/4 pi/3 pi/2])
       xticklabels({'-\pi/2','-\pi/3','-\pi/4','-\pi/8','0','\pi/8','\pi/4','\pi/3','\pi/2'})
       set(gca,'TickDir','out')
       hold on, scatter(omega_anchors,r_anchors,30,'r','filled');
       hold on, scatter(Os(r_b),Rs(c_b),15,'m');
       hold off;

       figure(1);       
       patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
       patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);
       hold on, plot(pos_x,pos_y,'LineWidth',0.5,'Color',[1 0.1 0.8 0.5]);
       hold on,scatter(pos_x,pos_y,10,[1:numel(pos_y)],'filled');
       
       clf;
       
   end
 

  % empty the current trajectory posx and posy variables
   pos_x=[]; pos_y=[];
  % replace the new vectors with the next trajectory xy points
  pos_x=pos_xnew;
  pos_y=pos_ynew;

 % set current actions to the new ones
  rs_=rs_new;
  omegas_=omegas_new;

end

%% 
r_pop_avg(agent,:) = r_loop;
hd_pop_avg(agent,:)=om_main;
corr_pop_avg(agent,:) = target_hit;
anchors_no_pop(agent,:)=anchors_no;
r_anchors_pop{agent}=r_anchors_struct;
om_anchors_pop{agent}=omega_anchors_struct;
tol_radii(agent,:)=tol_vec;
I_agents(agent,:)=Ivec;
caching_times(agent,:)=caching_times_agent;

%posteriors(agent,:,:,:)=posteriors_per_agent;


end

toc
