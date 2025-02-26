
function [optimal_para,fval,exitflag,output,lambda,grad,hessian,optimizer_obj]=optimize_path_length_and_smoothness(r,theta,phi0_i,...
                                                             arena,rho,tol_radius,rg_r,rg_th,opts,w1,w2,dt,r_home)
% OPTIMIZE_PATH_LENGTH_AND_SMOOTHNESS - Optimizes the path length and smoothness of a trajectory.
%
% This function uses constrained optimization to find the optimal radial distances (r),
% angles (theta), and heading offsets (phi0_i) for a set of anchor points. The optimization
% minimizes a loss function (trajectory path length + total angular changes at anchors points ) while respecting constraints
% on the radii, angles, and nonlinear constraints around the anchor points.
%
% Inputs:
%   - r: Initial radial distances for anchor points.
%   - theta: Initial angles for anchor points.
%   - phi0_i: Initial heading offsets measured from heading vectors.
%   - arena: Dimensions of the arena.
%   - rho: scaling factor of the time as function of distance.
%   - tol_radius: Tolerance for radial deviations around anchor points.
%   - rg_r: Range of radial distances.
%   - rg_th: Range of angles.
%   - opts: Optimization options (e.g., algorithm, max iterations).
%   - w1              - weight of the path length in the objective function
%   - w2              - weight of the heading changes at the anchors in the
%   objective function.
%   - dt: Time step for trajectory evaluation.
%   - r_home: Minimum allowed radius (e.g., home position).
%
% Outputs:
%   - optimal_para: Optimized parameters (radii, angles, and heading offsets).
%   - fval: Value of the objective function at the optimized parameters.
%   - exitflag: Indicates the success or failure of the optimization.
%   - output: Additional information about the optimization process.
%   - lambda: Lagrange multipliers for constraints.
%   - grad: Gradient of the objective function.
%   - hessian: Hessian of the objective function.
%   - optimizer_obj: Optimizer object used for computations.
%
% Key Steps:
%   1. Define the number of anchor points and optimization parameters.
%   2. Generate initial parameter vector with slight perturbations (jittering).
%   3. Set lower and upper bounds for optimization parameters.
%   4. Perform constrained optimization using fmincon.
%   5. Evaluate the loss function with the optimized parameters.

%%
% Number of anchor points:
no_anchors=numel(r);

% Total number of parameters to optimize:
    % [2*n + n-1] where:
    % - 2*n: r and theta for each anchor.
    % - (n-1): offset angles for heading vectors between anchor pairs.

q= (2*no_anchors)+(no_anchors-1);

% Initialize lower and upper bounds for optimization parameters:
lb=zeros(1,q);
ub=zeros(1,q);
r0=r; % Initial radial distances
theta0=theta; % Initial angles.

% % Generate initial parameter vector with slight perturbations (jittering) 
% around the initial anchors radii and angles
% within the allowed radius of tolerance:
[r_i,theta_i]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home);

% Initial parameter vector: [phi0_i, r_i, theta_i]
p0=[phi0_i,r_i, theta_i];

%% lower bounds on parameters:

% Lower bound for the first heading vector offset:
lb(1)=-( pi-(theta(2)) );
lb(2:no_anchors-1)= -pi;   % offsets off heading vectors.
% Lower bounds for radial distances and angles:
lb(no_anchors:2*no_anchors-1)=repmat(r_home,1,no_anchors); % Radii >= r_home.
lb(2*no_anchors-1)=0; % last anchor radius (Home) = 0.
lb(no_anchors)=0; % First anchor radius (Home)= 0.
lb(2*no_anchors:end)=zeros(1,no_anchors); % Angles >= 0.

%% upper bounds on parameters:

% Upper bound for the first heading vector offset:
ub(1)=theta(2); % Constrained by the angle of the second anchor.
ub(2:no_anchors-1)=pi; 
% Upper bounds for radial distances and angles:
ub(no_anchors:2*no_anchors-1)=repmat(rg_r,1,no_anchors);  % Radii <= rg_r.
ub(2*no_anchors:end)=repmat(pi,1,no_anchors); % Angles <= pi.
ub(2*no_anchors-1)=0; % last anchor radius (Home) = 0.
ub(no_anchors)=0; % First anchor radius (Home)= 0.

%%
% Initialize the optimizer object:
optimizer_obj=OptimizerClass_smoothness_and_PL( );

% Perform constrained optimization using fmincon:
[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)optimizer_obj.loss(p,arena,rho,no_anchors,w1,w2,dt) ...
    ,p0,[],[],[],[],lb,ub,@(p)non_linear_const_around_anchors(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th,arena),opts);

% Evaluate the loss function with the optimized parameters:
[~]=optimizer_obj.loss(optimal_para,arena,rho,no_anchors,w1,w2,dt);

end

%%

% Non-linear constraint function to enforce a circular tolerance region around each anchor
% in polar coordinates (r, theta) and ensure the path stays within arena boundaries.

function [c,ceq]=non_linear_const_around_anchors(p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th,arena)

% Equality constraints (not used here, so empty):
ceq=[];

% Normalize the radial and angular differences between the current and initial anchors:
r_norm=(p(no_anchors:2*no_anchors-1)-initial_anchors_r)./rg_r;
th_norm=(p(2*no_anchors:end)-initial_anchors_theta)./rg_th;

% First non-linear constraint: Ensure the anchor points stay within a circular tolerance region.
% The constraint is: (r_norm)^2 + (th_norm)^2 <= tol_radius^2.
c1= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2); % Violation if > 0.

% Second non-linear constraint: Ensure the anchor points stay within the arena boundaries.
% Define angles corresponding to the arena's walls:

theta1=[atan2(arena(3),arena(2)),... % Right vertical wall angles.
    atan2(arena(4),arena(2))];

theta3=[atan2(arena(4),arena(1)),... % Left vertical wall angles.
    atan2(arena(3),arena(1))];

theta2=[atan2(arena(4),arena(2)),... % Top horizontal wall angles.
    atan2(arena(4),arena(1))];

% Initialize the second constraint vector:
c2=zeros(size(c1));

% Check each anchor point against the arena boundaries:
for n=1:no_anchors
    theta_n=p((2*no_anchors-1)+n) ; % Current anchor's angle.
    r_n=p(no_anchors-1+n); % Current anchor's radius.

    % Right vertical wall constraint:
    if theta_n>theta1(1) && theta_n<=theta1(2)  
             c2(n)= r_n- (arena(2)./cos(theta_n)); % Violation if > 0.
    % Left vertical wall constraint:
    elseif theta_n>theta3(1) && theta_n<=theta3(2)  
             c2(n)= r_n- (arena(1)./cos(theta_n)); % Violation if > 0.

     % Top horizontal wall constraint:         
    elseif  theta_n>theta2(1) && theta_n<=theta2(2)
             c2(n)= r_n- (arena(4)./sin(theta_n)); % Violation if > 0.
    end
    
end

% Combine the two constraints and take the maximum violation for each anchor:
c=max([c1',c2'],[],2); % Ensure the output is a column vector.

end

%%

function [r_jitt,theta_jitt]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home)

% This function generates slightly perturbed (jittered) versions of the initial anchor points (r0, theta0) while ensuring that the jittered points:
% 
% Stay within a circular tolerance region around the initial points.
% 
% Stay within the boundaries of the arena.
% 
% Respect lower and upper bounds for radii and angles.

% Initialize the first anchor (home anchor) without jittering:
r_jitt(1)=r0(1); % Keep the home anchor's radius unchanged.
theta_jitt(1)=theta0(1); % Keep the home anchor's angle unchanged.

% Jitter all anchors except the home anchor and the last anchor:

for n=2:no_anchors-1
    
    % Keep generating jittered points until they satisfy the constraints:

    while(1)
    
        % Add random noise to the radius and angle within the tolerance radius:
        r_jitt(n)=r0(n)+(randn(1)*tol_radius*rg_r); % Jitter the radius.
        theta_jitt(n)=theta0(n)+(randn(1)*tol_radius*rg_th); % Jitter the angle.

        % Ensure the jittered radius is within valid bounds:
        % If the jittered radius is negative, set it to 0:

        if (r_jitt(n)<0)
            r_jitt(n)=0;
        end

        % If the jittered radius is inside the home region, set it to r_home:

        if r_jitt(n)<r_home
            r_jitt(n)=r_home;
        end

        % Ensure the jittered angle is within valid bounds:
        % If the jittered angle is greater than pi, set it to just below pi:

        if(theta_jitt(n)>=pi)
            theta_jitt(n)=pi-(1e-2);
        end

        % If the jittered angle is less than 0, set it to just above 0:

        if (theta_jitt(n)<=0)
            theta_jitt(n)=0+(1e-2);
        end

        % Condition 1: Ensure the jittered point stays within the tolerance radius:

         r_norm=(r_jitt(n)-r0(n))./rg_r; % Normalized radial difference.
         th_norm=(theta_jitt(n)-theta0(n))./rg_th; % Normalized angular difference.
         c1= ( r_norm .^2)+...
        (th_norm.^2)-(tol_radius^2); % Violation if > 0.

        % Condition 2: Ensure the jittered point stays within the arena boundaries:

        % Define angles corresponding to the arena's walls:

        theta1=[atan2(arena(3),arena(2)),... % Right vertical wall angles.
            atan2(arena(4),arena(2))];
        
        theta3=[atan2(arena(4),arena(1)),... % Left vertical wall angles.
            atan2(arena(3),arena(1))];
        
        theta2=[atan2(arena(4),arena(2)),... % Top horizontal wall angles.
            atan2(arena(4),arena(1))];

        % Check which wall the jittered angle corresponds to

        if theta_jitt(n)>theta1(1) && theta_jitt(n)<=theta1(2)  
                 c2= r_jitt(n)- (arena(2)./cos(theta_jitt(n))); % Right wall constraint.

        elseif theta_jitt(n)>theta3(1) && theta_jitt(n)<=theta3(2)  
                 c2= r_jitt(n)- (arena(1)./cos(theta_jitt(n))); % Left wall constraint.

        elseif  theta_jitt(n)>theta2(1) && theta_jitt(n)<=theta2(2)
                 c2= r_jitt(n)- (arena(4)./sin(theta_jitt(n))); % Top wall constraint.
       end

       % If the jittered point satisfies both conditions, break the loop:

       if((c1<0)&&(c2<0))
           break
       end

    end

end
% Add the last anchor (home anchor) without jittering:
r_jitt(end+1)=r0(end); % Keep the last anchor's radius unchanged.
theta_jitt(end+1)=theta0(end); % Keep the last anchor's angle unchanged.


end

