% offline calculation of the amount of drift in the likelihood
clear all
clc
close all

%setup the task variables
arena_size=[-400 400 0 600];
arena.x=[arena_size(1) arena_size(2) arena_size(2) arena_size(1)];
arena.y=[arena_size(3) arena_size(3) arena_size(4) arena_size(4)];
env=environment; 
env.arena_dimensions= arena_size;  
sigma_ridge=20; %uncertainity in the value of the action parameters executed.
ags=50;   %number of agents to run
n=500;    %number of samples in speed space and angle space.
max_speed=1000;  %maximum speed value in the action space.
min_speed=1;     % minimum speed value in the action space
max_angle=pi/2;  %maximum angle in the action space (relative to the vertical axis) 
min_angle=-pi/2;   %minimum angle in the action space (relative to the vertical axis)
speed_step=round((max_speed-min_speed)/n);
angle_step=((max_angle-min_angle)/n);
As=linspace(min_speed,max_speed,n);
Os=linspace(min_angle,max_angle,n);
clearnce=0.5; %loop width in radians 
T=50;
c_drift=0;
wrkrs=4;


[prior,r_bounds,c_bounds]= set_control_actions_space(As,Os,env.arena_dimensions,clearnce,T);

% for every action in the arena, compute the likelihood and drift from the
% anchor position
Drift={};
tic
for ang=1:numel(Os) 
    temp_cols_boundaries=c_bounds(find(r_bounds==ang));
    ang
    parfor sp=1:500
        
         notboundary_cond= (sp<=max(temp_cols_boundaries));
        
         if(notboundary_cond)

         mu_anchors=[0 As(sp) 0];
         omega_anchors=[Os(ang), Os(ang), Os(ang)];
         %% the generative model connecting anchors with a smooth trajectory 
         [mus_,omegas_,pos_x,pos_y]= connect_actions_with_smooth_trajectory_wo_noise(mu_anchors,omega_anchors,sigma_ridge,speed_step,env,clearnce,c_drift,T);
         [posterior]=Bayes_update_for_actions_params(1,mus_,omegas_,sigma_ridge,As,Os,prior,wrkrs);

         % linear index of the maximum in the posterior
         [~,i]= max(posterior(:));
         [r_max,c_max]= ind2sub(size(posterior),i);
         a_c_max=As(c_max);
         o_r_max=Os(r_max);

         % compute the drift= distance between anchor of maximum Likelihood
         % and the actual anchor in angle and speed independently
         drift_speed= ( (a_c_max-  mu_anchors(2)) );
         drift_angle= sqrt( (o_r_max-omega_anchors(2))^2);
         Drift{ang,sp}= [drift_angle, drift_speed];
        
        else
            Drift{ang,sp}=[nan nan];
        end
         

    end
    

end
toc
% i=250;
% plot_drift_speed_for_angle(Drift,i,As,Os);















