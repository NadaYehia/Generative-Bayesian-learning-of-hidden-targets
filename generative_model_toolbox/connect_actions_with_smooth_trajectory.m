function [mu,omega,x_op,y_op]=connect_actions_with_smooth_trajectory(mu_anchors,omega_anchors,sigma_ridge,speed_step,env)

T=1000;
k=(T)/pi;
arena=env.arena_dimensions;
a=2;
c=0; %1.6
speed_noise=gamrnd(a*(c*sigma_ridge*speed_step),1/a);
% speed_noise=0;

x_op=[];
y_op=[];
x_=[];
y_=[];
% for every anchor, sum the xand y to the maximum points and this will be
% x_and y_

for f=2:numel(mu_anchors)-1

    theta= (omega_anchors(f)) +(pi/2);

    r=1*T*(cos(theta))* (mu_anchors(f)+speed_noise);
    q=k* (4 ); 
    x_(f)=r/q;

    l=1*T*(sin(theta))* (mu_anchors(f)+speed_noise);
    m=k* ( 4 ); 
    y_(f)=l/m;

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

    % first compute the normalized ecul. distance between anchors n and n+1
    heading_offset= (atan2( y_(n+1)-y_(n),x_(n+1)-x_(n) ));
    dx_=x_(n+1)-x_(n);
    dy_=y_(n+1)-y_(n);

    r1=k*dy_* ( 4 );
    q1= 1*T*(sin(heading_offset));
    vmax_y= r1/q1;

    r=k*dx_* ( 4 );
    q= 1*T*(cos(heading_offset));
    vmax_x= r/q;

    if(dx_==0)
        vmax=vmax_y;
    else
        vmax=vmax_x;
    end

    % use the generative model to connect neighbor anchors
    w=(2*pi)/(T);
    t1=[0:T/2];
    speed= [sin(w.*t1)];
    speed=speed./sum(speed);
    speed= (vmax).*speed;

     heading= ( ((2*pi)/T) .*t1)+( (heading_offset-(pi/2)));

    pos_x(1)=x_(n);
    pos_y(1)=y_(n);
    
    for t=2:size(heading,2)
    [dx,dy] = pol2cart(heading(t),speed(t));
     pos_x(t)=pos_x(t-1)+dx;

        if pos_x(t)<arena(1)
            pos_x(t)=arena(1);                
        elseif pos_x(t)>arena(2)
            pos_x(t)=arena(2);
        end

         pos_y(t)=pos_y(t-1)+dy;

        if pos_y(t)<arena(3)
            pos_y(t)=arena(3);                
        elseif pos_y(t)>arena(4)
            pos_y(t)=arena(4);
        end
    
    end

x_op=[x_op, pos_x(1:end)];
y_op=[y_op, pos_y(1:end)];


pos_y=[];
pos_x=[];

end
 [mu,omega]= convert_xy_velo_angle(x_op,y_op);


end