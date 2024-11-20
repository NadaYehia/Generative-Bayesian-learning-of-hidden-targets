function [r,omega,x_op,y_op]=connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Os,Rs,bestDataFitScaleOffset,bestDataFitMeanToScaleRatio,...
    angle_noise_scale,ka,w20,tol_radius)

kd=zeros(1,numel(r_anchors)-1);
kd(1)=clearnce;
rg_r=abs(Rs(end)-Rs(1));
rg_th=abs(Os(end)-Os(1));
exitflag=0;
tto=0;
MaxTrials=15;
dt=0.001;
x_op=[];
y_op=[];
x_=[];
y_=[];
arena=env.arena_dimensions;
heading_conca=[];
speed_conca=[];

for f=2:numel(r_anchors)-1

    theta0(f)= ((omega_anchors(f)) +(pi/2)) +(randn(1)*angle_noise_scale);
    
    % fit to the data average sigma to mean ratio across mouse 1&2 on all
    % targets
    scale= bestDataFitScaleOffset+bestDataFitMeanToScaleRatio*r_anchors(f);
    % fit to the data average sigma to mean ratio across mouse 1&2 on all
    % targets 
    
    r0(f)= r_anchors(f)+(scale*randn(1));

    x_(f)=r0(f)*cos(theta0(f));
    y_(f)=r0(f)*sin(theta0(f));

end

x_(x_>arena(2))=arena(2);
x_(x_<arena(1))=arena(1);

y_(y_>arena(4))=arena(4);
y_(y_<arena(3))=arena(3);

x_(1)=0; y_(1)=0;
x_(end+1)=0; y_(end+1)=0;

% compute r and theta from the bounded x&y points.
r0=sqrt((x_.^2)+(y_.^2));
theta0=atan2(y_,x_);

% keep initial and final heading angles not being 0
theta0(1)=omega_anchors(1)+(pi/2);
theta0(end)=omega_anchors(end)+(pi/2);

%% 
for n=1:size(r0,2)-1
 
    vx=(-r0(n)*cos(theta0(n))) +(r0(n+1)*cos(theta0(n+1)));
    vy=(-r0(n)*sin(theta0(n))) +(r0(n+1)*sin(theta0(n+1)));

    heading_offset= (atan2(vy,vx ));
    
    % enforce continuity in the heading angle function:
    if(n~=1)
        % find the angle of curvature of the next part given the 
        % value of the last heading angle in the trajectory.

       kd(n)=wrapToPi(heading_offset-(heading_conca(end)));
 
        
        tol=abs(kd(n));
        k=sign(kd(n));
        epsi=((tol-pi/2));
    else
        %randomize the first anchor direction
        k=sign(kd(1));
        tol=abs(kd(1));
        epsi=((tol-pi/2));
    end
    
   
    eucl_dist=sqrt(vx^2 +vy^2);
    
    vmax_n=eucl_dist*((4*pi)+(4*epsi));
    vmax_d= (pi*ka)*(sinc(epsi/pi));

    vmax= sqrt(vmax_n/vmax_d);
    T=ka*vmax;
        if(~isreal(T))
            error('Time cant be complex, sinc function is outside pi and -pi');
        end
    
    % use the functional form to produce x,y points in space
    w=(2*pi)/(T);
    t1=[0:dt:T/2];
    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));

    heading=wrapToPi(heading);
    
    pos_x=zeros(1,size(heading,2));
    pos_y=zeros(1,size(heading,2));
    
    dx= zeros(1,size(heading,2));
    dy=zeros(1,size(heading,2));

    dx(1)=r0(n)*cos(theta0(n));
    dy(1)=r0(n)*sin(theta0(n));
    
    [temp_dx,temp_dy] = pol2cart(heading(2:end),speed(2:end).*dt);
    dx(2:end)=temp_dx;
    dy(2:end)=temp_dy;

    pos_x=cumsum(dx);
    pos_y=cumsum(dy);
 
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
% figure,scatter(x_op,y_op,30,[1:numel(x_op)]);
x_op_old=x_op;
y_op_old=y_op;
x_old=x_;
y_old=y_;
MFE=30000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,"EnableFeasibilityMode",true);
w2=w20;
ri=r0;
thetai=theta0;
%% optimize anchors for path length
while(exitflag<=0)

    [optimal_para,fval,exitflag,output,lambda,grad,hessian]=...
        optimize_path_length(ri,thetai,kd,arena,ka,w2,tol_radius,rg_r,rg_th,opts,dt);
    
    if(exitflag<=0 )
        %noise to the starting point, very small values
        ri=r0+0.0001*randn(1);
        thetai=theta0+0.0001*randn(1);
    end

    tto=tto+1;
     
    if(tto>MaxTrials)
        % return original solution w/o noise or anything, 
        optimal_para=[];
        optimal_para=[kd,r0,theta0];
        break;
    end
      
end
%% plot path after anchors optimization 
k_d0=optimal_para(1:numel(r0)-1);
temp_kd0=abs(wrapToPi(k_d0));



ancs_no=numel(r0);
r=optimal_para(ancs_no:ancs_no+ancs_no-1);
theta=optimal_para(ancs_no+ancs_no:end);
x_op=[];
y_op=[];
x_opc=[];
y_opc=[];
x_=[];
y_=[];
x_=r.*cos(theta);
y_=r.*sin(theta);
heading_conca=[];
speed_conca=[];

for n=1:size(r,2)-1
 
    vx=(-r(n)*cos(theta(n))) +(r(n+1)*cos(theta(n+1)));
    vy=(-r(n)*sin(theta(n))) +(r(n+1)*sin(theta(n+1)));

    heading_offset= (atan2(vy,vx ));

    % enforce continuity in the heading angle function:
    if(n~=1)
        % find the angle of curvature of the next part given the 
        % value of the last heading angle in the trajectory.
        k_d0(n)=wrapToPi(k_d0(n));
        tol=abs(k_d0(n));
        k=sign(k_d0(n));
        epsi=((tol-pi/2));
    else
        %
        k_d0(1)=wrapToPi(k_d0(1));
        k=sign(k_d0(n));
        tol=abs(k_d0(n));
        epsi=((tol-pi/2));
    end
    
   
    eucl_dist=sqrt(vx^2 +vy^2);
    
    vmax_n=eucl_dist*((4*pi)+(4*epsi));
    vmax_d= (pi*ka)*(sinc(epsi/pi));

    vmax= sqrt(vmax_n/vmax_d);
    T=ka*vmax;
        if(~isreal(T))
            error('Time cant be complex, sinc function is outside pi and -pi');
        end
    
    % use the functional form to produce x,y points in space
    w=(2*pi)/(T);
    t1=[0:dt:T/2];
    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));
    heading=wrapToPi(heading);
    
    pos_x=zeros(1,size(heading,2));
    pos_y=zeros(1,size(heading,2));
    
    dx= zeros(1,size(heading,2));
    dy=zeros(1,size(heading,2));

    
    if(isempty(x_op)) %no avoided obstacles or any loop adjustments
        dx(1)=r(n)*cos(theta(n));
        dy(1)=r(n)*sin(theta(n));

    else 
        dx(1)=x_op(end);
        dy(1)=y_op(end);
   
    end
    
    [temp_dx,temp_dy] = pol2cart(heading(2:end),speed(2:end).*dt);
    dx(2:end)=temp_dx;
    dy(2:end)=temp_dy;

    pos_x=cumsum(dx);
    pos_y=cumsum(dy);
 

    pos_x( find(pos_x>arena(2)) )=arena(2); 
    pos_x( find(pos_x<arena(1)) )=arena(1);
    pos_y( find(pos_y>arena(4)) )=arena(4); 
    pos_y( find(pos_y<arena(3)) )=arena(3); 

    %% handling obstacles
    [pos_xc,pos_yc]=avoid_obstacles(pos_x,pos_y,env);
    %% handling obstacles

% x_opc=[x_opc, pos_xc(1:end)];
% y_opc=[y_opc, pos_yc(1:end)];

x_op=[x_op, pos_xc(1:end)];
y_op=[y_op, pos_yc(1:end)];

speed_conca=[speed_conca,speed];
heading_conca=[heading_conca, heading];

pos_y=[];
pos_x=[];
pos_yc=[];
pos_xc=[];

end
% figure,scatter(x_op,y_op,30,[1:numel(x_op)])
%% plot paths and anchors before and after optimization
% figure,plot(x_op_old,y_op_old,'r');
% hold on, scatter(x_old,y_old,'r');
% hold on, plot(x_op,y_op,'b');
% hold on, scatter(x_,y_,'b');

% infer r,omegas from the optimized path
 [r,omega]= convert_xy_r_angle(x_op,y_op);
  


end