
function [optimal_para,fval,exitflag,output,lambda,grad,hessian]=optimize_path_length(r,theta,k_d0,...
                                                             arena,ka,w2,tol_radius,rg_r,rg_th,opts,dt)

no_anchors=numel(r);
phi=tol_radius*rg_th; %maximum angles margin in the tolerance.

% total number of parameters to optimize
q=numel(r)+numel(theta)+numel(k_d0);
lb=zeros(1,q);
ub=zeros(1,q);

p0=[k_d0,r, theta];
r0=r;
theta0=theta;
%%
lb(1:no_anchors-1)=-inf;

% curvature lower bound on the first anchor
lb(1)=-( pi-(theta(2)+phi) );
% curvature lower bound on the first anchor

lb_r_th=zeros(1,2*no_anchors); %radius can't be negative 

lb(no_anchors:end)=lb_r_th;

%%
ub(1:no_anchors-1)=inf;

%curvature upper bound on the first anchor
ub(1)=theta(2)-phi; 
%curvature upper bound on first anchor
ub_r_th(1:no_anchors)=repmat(rg_r,1,no_anchors);

ub_r_th(no_anchors+1:2*no_anchors)=pi;

ub(no_anchors:end)=ub_r_th;

[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)my_path_length(p,arena,ka,no_anchors,w2,dt),p0,[],[],[],[],lb,ub,@(p)My_r_theta_circle_cons(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th),opts);

end

function total_cost=my_path_length(p,arena,ka,no_anchors,w2,dt)
if(any(isnan(p)))
    flgg=1;
end
x_op=[];
y_op=[];
speed_conca=[];
heading_conca=[];


k_d0=p(1:no_anchors-1);
s=no_anchors; 
r=p(s:s+no_anchors-1);
s=no_anchors+no_anchors;
theta=p(s:end);

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
        %randomize the first anchor direction
        k_d0(1)=wrapToPi(k_d0(1));
        k=sign(k_d0(1));
        tol=abs(k_d0(1));
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
    
    % duration of this segment
    Ts(n)= (t1(end));

    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));
    heading=wrapToPi(heading);
    
    pos_x(1)=r(n)*cos(theta(n));
    pos_y(1)=r(n)*sin(theta(n));
    
    for t=2:size(heading,2)
        [dx,dy] = pol2cart(heading(t),speed(t)*dt);
        pos_x(t)=pos_x(t-1)+dx;

        pos_y(t)=pos_y(t-1)+dy;

    end
    pos_x( find(pos_x>arena(2)) )=arena(2); 
    pos_x( find(pos_x<arena(1)) )=arena(1);
    pos_y( find(pos_y>arena(4)) )=arena(4); 
    pos_y( find(pos_y<arena(3)) )=arena(3); 

if (n~=1)
    domega=wrapToPi(heading(1)-heading_conca(end));
    K(n)=abs(domega);
end

x_op=[x_op, pos_x(1:end)];
y_op=[y_op, pos_y(1:end)];
speed_conca=[speed_conca,speed];
heading_conca=[heading_conca, heading];

pos_y=[];
pos_x=[];


end

Pl=sum_path_length(x_op,y_op);  
total_path_duration=numel(x_op)*dt;
kappa=sum(K);

total_cost= (Pl)+w2*( kappa);
if(isnan(Pl))
    error('path length is nan')
end

end

function PL=sum_path_length(x,y)
d_vx=diff(x);
d_vy=diff(y);

PL=sum(vecnorm([d_vx' d_vy'],2,2));

%  for i=1:size(x,2)-1
%      vx=x(i+1)-x(i);
%      vy=y(i+1)-y(i);
%      PL=PL+norm([vx,vy]);
% 
%  end

end

function [c,ceq]=My_r_theta_circle_cons (p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th)
    
ceq=[];
r_norm=(p(no_anchors:(2*no_anchors)-1)-initial_anchors_r)./rg_r;
th_norm=(p(2*no_anchors:end)-initial_anchors_theta)./rg_th;
c= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2);

end

