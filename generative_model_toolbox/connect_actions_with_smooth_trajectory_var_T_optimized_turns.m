function [mu,omega,x_op,y_op]=connect_actions_with_smooth_trajectory_var_T_optimized_turns(mu_anchors,omega_anchors,sigma_ridge,speed_step,env,clearnce,Drift,Os,As,bestDataFitScaleOffset,bestDataFitMeanToScaleRatio,...
    angle_noise_scale,drift_fac,ka)

tol=clearnce;
arena=env.arena_dimensions;
heading_conca=[];
speed_conca=[];

for ad=2:numel(mu_anchors)-1
    i=find(Os==omega_anchors(ad));
    j= find(As==mu_anchors(ad));
    drifts=Drift{i,j};
    drift_speed=drifts(2);
    mu_anchors(ad)=mu_anchors(ad)+(drift_fac*drift_speed); % the offseted anchor value
end

x_op=[];
y_op=[];
x_=[];
y_=[];
 
epsi=(tol-pi/2);

for f=2:numel(mu_anchors)-1

    theta= ((omega_anchors(f)) +(pi/2)) +(randn(1)*angle_noise_scale);
    
    % fit to the data average sigma to mean ratio across mouse 1&2 on all
    % targets
    scale= bestDataFitScaleOffset+bestDataFitMeanToScaleRatio*mu_anchors(f);
    % fit to the data average sigma to mean ratio across mouse 1&2 on all
    % targets 
    
    noisy_corrected_anchor= mu_anchors(f)+(scale*randn(1));
    speed_noise(f)=noisy_corrected_anchor-mu_anchors(f);
    
   
    T=ka*noisy_corrected_anchor;
    E_d= (( noisy_corrected_anchor)*(pi*T)*sinc(epsi/pi)) / ((4*pi)+(4*(epsi)));

    x_(f)=E_d*cos(theta);

    y_(f)=E_d*sin(theta);

end

x_(x_>arena(2))=arena(2);
x_(x_<arena(1))=arena(1);

y_(y_>arena(4))=arena(4);
y_(y_<arena(3))=arena(3);

x_(1)=0; y_(1)=0;
x_(end+1)=0; y_(end+1)=0;


% run the noisy version of the trajectory and save these x,y points
% n links, connecting n anchors, return the noisy executed trajectory.
for n=1:size(mu_anchors,2)-1

    heading_offset= (atan2( y_(n+1)-y_(n),x_(n+1)-x_(n) ));

    % enforce continuity in the heading angle function:
    if(~isempty(heading_conca))
        % find the angle of curvature of the next part given the 
        % value of the last heading angle in the trajectory.

        k_tol=(heading_offset)-heading_conca(end);
        tol=abs(k_tol);
        k=sign(k_tol);
        epsi=(tol-pi/2);
    else
        %randomize the first anchor direction
        p=rand(1);
        if(p<0.5)
            k=1;
        else
            k=-1;
        end
        epsi=(tol-pi/2);
    end
    
    dx_=x_(n+1)-x_(n);
    dy_=y_(n+1)-y_(n);
    eucl_dist=sqrt(dx_^2 +dy_^2);
    
    vmax_n=eucl_dist*((4*pi)+(4*epsi));
    vmax_d= (pi*ka)*(sinc(epsi/pi));

    vmax= sqrt(vmax_n/vmax_d);
    T=ka*vmax;
    dt=0.01;
    % use the functional form to produce x,y points in space
    w=(2*pi)/(T);
    t1=[0:dt:T/2];
    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));

    pos_x(1)=x_(n);
    pos_y(1)=y_(n);
    
    for t=2:size(heading,2)
        [dx,dy] = pol2cart(heading(t),speed(t)*dt);
        pos_x(t)=pos_x(t-1)+dx;

        pos_y(t)=pos_y(t-1)+dy;

    end
    pos_x( find(pos_x>arena(2)) )=arena(2); 
    pos_x( find(pos_x<arena(1)) )=arena(1);
    pos_y( find(pos_y>arena(4)) )=arena(4); 
    pos_y( find(pos_y<arena(3)) )=arena(3); 



x_op=[x_op, pos_x(1:end)];
y_op=[y_op, pos_y(1:end)];
speed_conca=[speed_conca,speed];
heading_conca=[heading_conca, heading];
pos_y=[];
pos_x=[];

end
 [mu,omega]= convert_xy_velo_angle(x_op,y_op,clearnce,ka);
 %% 


end