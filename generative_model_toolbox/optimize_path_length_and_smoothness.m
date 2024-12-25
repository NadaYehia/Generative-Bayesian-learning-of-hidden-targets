
function [optimal_para,fval,exitflag,output,lambda,grad,hessian,optimizer_obj]=optimize_path_length_and_smoothness(r,theta,k_d0,...
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
optimizer_obj=OptimizerClass( );

[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)optimizer_obj.my_loss(p,arena,ka,no_anchors,w2,dt) ...
    ,p0,[],[],[],[],lb,ub,@(p)My_r_theta_circle_cons(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th),opts);

[~]=optimizer_obj.my_loss(optimal_para,arena,ka,no_anchors,w2,dt);

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

