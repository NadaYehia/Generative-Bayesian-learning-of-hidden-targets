
clear all
clc
% setup the environment and action space
[intercept,sigma_ridge,arena,blocks,target,target2]=env_settings('anypt',20,[60 150],10,250,-250,250,50,50,50,50);

% set some initial variables.
ags=50;
speeds=[1:2:1000];
angles=[-pi/2:2*pi/1000:pi/2];
sampler='proportional'; 
% sampler='peak_sampler';
draw_flg=0;
target_num=2;

dist_criterion=func_sigma_ridge_dist(1,sigma_ridge);
win=1;
tic
% start agents trials
parfor agent=1:ags
 
 initial_ancs=10;
 
 
 [prior,As,Os,speed_step,r_bounds,c_bounds]= set_control_actions_space(speeds,...
    angles,arena);

 [mu_spd,om_main,target_hit,anchors_no,mu_anchors_variance,...
         omega_anchors_variance,anchors_no_variance,posterior_support_mu_var,posterior_support_omega_var,...
          posterior_support_mu_entropy,posterior_support_omega_entropy]=    run_a_random_agent_trial_on_a_target (blocks,arena,target,target2,target_num,r_bounds,c_bounds,prior,initial_ancs,As,Os,sampler,...
                                               dist_criterion,sigma_ridge,speed_step,draw_flg)

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
