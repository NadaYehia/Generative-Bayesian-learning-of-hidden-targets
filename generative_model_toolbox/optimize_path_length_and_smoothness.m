
function [optimal_para,fval,exitflag,output,lambda,grad,hessian,optimizer_obj]=optimize_path_length_and_smoothness(r,theta,k_d0,...
                                                             arena,ka,w1,tol_radius,rg_r,rg_th,opts,dt,r_home)

no_anchors=numel(r);

% total number of parameters to optimize: [2*n +n-1] (r and theta of n anchors)
% + (offset angles of the (n-1) heading vectors between every pair of anchors) 

q= (2*no_anchors)+(1);

% initialize the (1xq) row vector of upper and lower bounds of optimization to 0 
lb=zeros(1,q);
ub=zeros(1,q);
r0=r;
theta0=theta;

% initial value of the parameter vector P0
[r_i,theta_i]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home);

p0=[k_d0,r_i, theta_i];

%% lower bounds on parameters:

% lower bound on the first heading vector offset: home anchor to first
% anchor
lb(1)=-( pi-(theta(2)) );

% lower bound for any anchor angle=0, home anchors radii=0, and radius of all anchors
% 2nd to the second last is =r_home.

lb(2:2+(no_anchors-1))=repmat(r_home,1,no_anchors);
lb(2+no_anchors-1)=0;
lb(2)=0;
lb(2+no_anchors:end)=zeros(1,no_anchors);

%% upper bounds on parameters:

% upper bound on the first heading vector offset: home anchor to first
% anchor
ub(1)=theta(2); 

% upper bounds for any anchor angle and radius is R and pi respec.
ub(2:2+(no_anchors-1))=repmat(rg_r,1,no_anchors);
ub(2+no_anchors:end)=repmat(pi,1,no_anchors);
ub(2+no_anchors-1)=0;
ub(2)=0;
%
optimizer_obj=OptimizerClass( );

[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)optimizer_obj.my_loss(p,arena,ka,no_anchors,w1,dt) ...
    ,p0,[],[],[],[],lb,ub,@(p)My_r_theta_circle_cons(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th,arena),opts);

[~]=optimizer_obj.my_loss(optimal_para,arena,ka,no_anchors,w1,dt);

end



% Non-linear constraint of a circle with radius (tol_radius), an error radius, in normalized
% units) around every anchor in r,theta space.

function [c,ceq]=My_r_theta_circle_cons (p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th,arena)

ceq=[];
r_norm=(p(2:2+(no_anchors-1))-initial_anchors_r)./rg_r;
th_norm=(p(2+no_anchors:end)-initial_anchors_theta)./rg_th;
c1= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2);

% second non-linear constraint (right vertcial wall, left vertical wall
% top wall)

theta1=linspace(atan2(arena(3),arena(2)),...
    atan2(arena(4),arena(2)),100);

theta3=linspace(atan2(arena(4),arena(1)),...
    atan2(arena(3),arena(1)),100);

theta2=linspace(atan2(arena(4),arena(2)),...
    atan2(arena(4),arena(1)),100);

c2=zeros(size(c1));

for n=1:no_anchors
    theta_n=p((2+no_anchors-1)+n) ;
    r_n=p(1+n);
    if theta_n>theta1(1) && theta_n<=theta1(end)  
             c2(n)= r_n- (arena(2)./cos(theta_n));
    
    elseif theta_n>theta3(1) && theta_n<=theta3(end)  
             c2(n)= r_n- (arena(1)./cos(theta_n));
    elseif  theta_n>theta2(1) && theta_n<=theta2(end)
             c2(n)= r_n- (arena(4)./sin(theta_n));
    end
    
end

c=max([c1',c2'],[],2);

end


function [r_jitt,theta_jitt]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home)

r_jitt(1)=r0(1);
theta_jitt(1)=theta0(1);

%start jittering all anchors except home anchors.
for n=2:no_anchors-1
    
    %check if the jittered points are infeasible
    while(1)
    
        r_jitt(n)=r0(n)+(randn(1)*tol_radius*rg_r);
        theta_jitt(n)=theta0(n)+(randn(1)*tol_radius*rg_th);

        % make sure the jittered anchors are within the lower&upper bounds
        % of angles and radii ranges
        % 
        %if jittered anchor radius is negative, set it to 0
        if (r_jitt(n)<0)
            r_jitt(n)=0;
        end
        % if jittered anchor radius is inside the home region
        if r_jitt(n)<r_home
            r_jitt(n)=r_home;
        end

        %if jittered anchor angle is greater than pi or less than 0

        %set it to ~ pi
        if(theta_jitt(n)>=pi)
            theta_jitt(n)=pi-(1e-2);
        end

        %set it to ~ 0
        if (theta_jitt(n)<=0)
            theta_jitt(n)=0+(1e-2);
        end

        % Condition 1: tolerance radius around the initial points
         r_norm=(r_jitt(n)-r0(n))./rg_r;
         th_norm=(theta_jitt(n)-theta0(n))./rg_th;
         c1= ( r_norm .^2)+...
        (th_norm.^2)-(tol_radius^2);

        %Condition 2: the arena boundaries
        theta1=linspace(atan2(arena(3),arena(2)),...
            atan2(arena(4),arena(2)),100);
        
        theta3=linspace(atan2(arena(4),arena(1)),...
            atan2(arena(3),arena(1)),100);
        
        theta2=linspace(atan2(arena(4),arena(2)),...
            atan2(arena(4),arena(1)),100);
        
        if theta_jitt(n)>theta1(1) && theta_jitt(n)<=theta1(end)  
                 c2= r_jitt(n)- (arena(2)./cos(theta_jitt(n)));
        elseif theta_jitt(n)>theta3(1) && theta_jitt(n)<=theta3(end)  
                 c2= r_jitt(n)- (arena(1)./cos(theta_jitt(n)));
        elseif  theta_jitt(n)>theta2(1) && theta_jitt(n)<=theta2(end)
                 c2= r_jitt(n)- (arena(4)./sin(theta_jitt(n)));
       end

       % if the jittery points satisfy both conditions then break and 
       % jitter the next anchor.
       if((c1<0)&&(c2<0))
           break
       end

    end

end
r_jitt(end+1)=r0(end);
theta_jitt(end+1)=theta0(end);

end

