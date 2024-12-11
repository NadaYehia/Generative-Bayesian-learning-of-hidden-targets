
function [optimal_para,fval,exitflag,output,lambda,grad,hessian]=optimize_path_length_and_smoothness(r,theta,k_d0,...
                                                             arena,ka,w2,tol_radius,rg_r,rg_th,opts,dt)

no_anchors=numel(r);

% total number of parameters to optimize: [2*n +n-1] (r and theta of n anchors)
% + (offset angles of the (n-1) heading vectors between every pair of anchors) 

q= (2*no_anchors)+(no_anchors-1);

% initialize the (1xq) row vector of upper and lower bounds of optimization to 0 
lb=zeros(1,q);
ub=zeros(1,q);

% initial value of the parameter vector P0
p0=[k_d0,r, theta];
r0=r;
theta0=theta;

%% lower bounds on parameters:
lb(1:no_anchors-1)=-inf;

% lower bound on the first heading vector offset: home anchor to first
% anchor
lb(1)=-( pi-(theta(2)) );

% lower bound for any anchor angle and radius =0.
lb_r_th=zeros(1,2*no_anchors);  
lb(no_anchors:end)=lb_r_th;

%% upper bounds on parameters:
ub(1:no_anchors-1)=inf;

% upper bound on the first heading vector offset: home anchor to first
% anchor
ub(1)=theta(2); 

% upper bounds for any anchor angle and radius is R and pi respec.
ub_r_th(1:no_anchors)=repmat(rg_r,1,no_anchors);
ub_r_th(no_anchors+1:2*no_anchors)=pi;
ub(no_anchors:end)=ub_r_th;

% 
[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)my_loss(p,arena,ka,no_anchors,w2,dt),p0,[],[],[],[],lb,ub,@(p)My_r_theta_circle_cons(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th),opts);

end

function total_cost=my_loss(p,arena,ka,no_anchors,w2,dt)


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

    if(n~=1)
       
        k_d0(n)=wrapToPi(k_d0(n));
        tol=abs(k_d0(n));
        k=sign(k_d0(n));
        epsi=((tol-pi/2));
         
    else
        
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
  
    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));
    heading=wrapToPi(heading);

% calculate the x&y points of a trajectory segment
    pos_x=zeros(1,size(heading,2));
    pos_y=zeros(1,size(heading,2));
    dx= zeros(1,size(heading,2));
    dy=zeros(1,size(heading,2));
    dx(1)=r(n)*cos(theta(n));
    dy(1)=r(n)*sin(theta(n));
    [temp_dx,temp_dy] = pol2cart(heading(2:end),speed(2:end).*dt);
    dx(2:end)=temp_dx;
    dy(2:end)=temp_dy;
    pos_x=cumsum(dx);
    pos_y=cumsum(dy);

% confine the trajectory segment to the arena enclosure
    pos_x( find(pos_x>arena(2)) )=arena(2); 
    pos_x( find(pos_x<arena(1)) )=arena(1);
    pos_y( find(pos_y>arena(4)) )=arena(4); 
    pos_y( find(pos_y<arena(3)) )=arena(3); 


% calculate the difference in heading angles at the anchor points.
% dOmega= theta at t=1 of the current segment - theta at t=T of the 
% previous segment.

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

% path length
Pl=sum_path_length(x_op,y_op);  
% sum of anglular changes at the anchor points
kappa=sum(K);
% loss function= path length+ angular changes at anchor points
total_cost= (Pl)+w2*( kappa);

% ERROR CHCK
if(isnan(Pl))
    error('path length is nan')
end

end

function PL=sum_path_length(x,y)
d_vx=diff(x);
d_vy=diff(y);

PL=sum(vecnorm([d_vx' d_vy'],2,2));
end

% Non-linear constraint of a circle with radius (tol_radius), an error radius, in normalized
% units) around every anchor in r,theta space.

function [c,ceq]=My_r_theta_circle_cons (p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th)

ceq=[];
r_norm=(p(no_anchors:(2*no_anchors)-1)-initial_anchors_r)./rg_r;
th_norm=(p(2*no_anchors:end)-initial_anchors_theta)./rg_th;
c= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2);

end

