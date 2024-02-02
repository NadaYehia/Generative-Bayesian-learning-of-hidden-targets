
clear all
clc
% setup the environment: the arena size, number of target, special objects
% (e.g. obstacles) *to be added*

targets_xy=[10 250; -250 250]; % 2Dcenters per target: nx2
targets_sizes=[60 60; 60 60];  % target sizes in x&y: nx2
arena=[-400 400 0 600];

env=environment;
env.intercept="anypt";
env.blocks=[60 150]; % number of trials per target: 1xn
env.targets_centers=targets_xy; 
env.targets_dimensions= targets_sizes; 
env.arena_dimensions= arena;  % arena size 1x4
targets=env.setup_targets_coord; % outputs nx2 struct: each row is a target
                                 % col1 are the x coords and col2 are the 
                                 % ycoords of the target corners


sigma_ridge=20; %uncertainity in the value of the action parameters executed.
ags=50;   %number of agents to run
n=500;    %number of samples in speed space and angle space.
max_speed=1000;  %maximum speed value in the action space.
min_speed=1;     % minimum speed value in the action space
max_angle=-pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=pi/2;   %minimum angle in the action space (relative to the vertical axis)

As=linspace(min_speed,max_speed,n);
Os=linspace(min_angle,max_angle,n);
res_x= round( (max_speed-min_speed)/n );
res_y= round( (max_angle-min_angle)/n );
range_x=max_speed-min_speed;
range_y=max_angle-min_angle;

sampler='proportional'; 
% sampler='peak_sampler';
draw_flg=1;
target_num=2;
k=1;
merging_criterion= pixel_dist_to_normal_eucl_dist(k,sigma_ridge,res_x,res_y,range_x,range_y);

[prior,r_bounds,c_bounds]= set_control_actions_space(As,Os,env.arena_dimensions);

win=1;
tic
% start agents trials
parfor agent=1:ags
 
 initial_ancs=10;
 
 
 [mu_spd,om_main,target_hit,anchors_no,mu_anchors_variance,...
         omega_anchors_variance,anchors_no_variance,posterior_support_mu_var,posterior_support_omega_var,...
          posterior_support_mu_entropy,posterior_support_omega_entropy]=    run_a_random_agent_trial_on_a_target (blocks,arena,target,target2,target_num,r_bounds,c_bounds,prior,initial_ancs,As,Os,sampler,...
                                               merging_criterion,sigma_ridge,res_x,draw_flg)











%% 
mu_pop_avg(agent,:) = mu_spd;
hd_pop_avg(agent,:)=om_main;
corr_pop_avg(agent,:) = target_hit;
anchors_no_pop(agent,:)=anchors_no;

% compute posterior support (as variance in its samples or entropy) to
% compare it with variance in executed anchors
dd= size(posterior_support_omega_entropy,2);
l=1;
var_x_mu_temp1=zeros(1,(dd-(2*win)));
var_x_mu_temp2=zeros(1,(dd-(2*win)));
var_y_mu_temp=zeros(1,(dd-(2*win)));
var_x_om_temp1=zeros(1,(dd-(2*win)));
var_x_om_temp2=zeros(1,(dd-(2*win)));
var_y_om_temp=zeros(1,(dd-(2*win)));


   for sc=win+1:dd-win
       var_x_mu_temp1(l)=(posterior_support_mu_var(sc));
       var_x_mu_temp2(l)=(posterior_support_mu_entropy(sc));
       
       var_y_mu_temp(l)= var( cell2mat(mu_anchors_variance(sc-win:sc+win)) );
       
       var_x_om_temp1(l)=(posterior_support_omega_var(sc));
       var_x_om_temp2(l)=(posterior_support_omega_entropy(sc));

       var_y_om_temp(l)= var( cell2mat(omega_anchors_variance(sc-win:sc+win)) );

       l=l+1;
   end

   var_x_mu(agent,:)=var_x_mu_temp1;
   entropy_x_mu(agent,:)=var_x_mu_temp2;
   var_y_mu(agent,:)=var_y_mu_temp;

   var_x_om(agent,:)=var_x_om_temp1;
   entropy_x_om(agent,:)=var_x_om_temp2;
   var_y_om(agent,:)=var_y_om_temp;



end

toc
