
function [optimal_para,fval,exitflag,output,lambda,grad,hessian,optimizer_obj]=optimize_path_length(r,theta,phi0_0,...
                                                             arena,rho,tol_radius,rg_r,rg_th,opts,dt,r_home)
% optimize_path_length - Optimizes the path length and angular changes for a trajectory through anchor points.
%                        The optimization minimizes a loss function while respecting constraints on the anchor points.
%
% Inputs:
%   r              - Radial coordinates of anchor points.
%   theta          - Angular coordinates of anchor points.
%   phi0_0         - Initial heading angle offset.
%   arena          - Arena dimensions (environment constraints).
%   rho            - scaling factor for total time T to execute a segment of length D. 
%   tol_radius     - Tolerance radius for constraints.
%   rg_r           - Range of radial values.
%   rg_th          - Range of angular values.
%   opts           - Optimization options for fmincon.
%   dt             - Time step for trajectory generation.
%   r_home         - Minimum allowed radial value (home position).
%
% Outputs:
%   optimal_para   - Optimized parameters (heading angles, radial and angular coordinates).
%   fval           - Value of the loss function at the optimized parameters.
%   exitflag       - Exit flag from the optimization process (indicates success or failure).
%   output         - Optimization output structure (e.g., iterations, best feasible solution).
%   lambda         - Lagrange multipliers at the solution.
%   grad           - Gradient of the loss function at the solution.
%   hessian        - Hessian of the loss function at the solution.
%   optimizer_obj  - Optimizer object containing intermediate results (e.g., optimized trajectory).


%%

% Determine the number of anchor points.
no_anchors=numel(r);

% Total number of parameters to optimize:
    % - 1 initial heading angle (phi0_0).
    % - Radial and angular coordinates for each anchor point (2 * no_anchors).
q= (2*no_anchors)+(1);

% Initialize lower and upper bounds for the optimization parameters.
lb=zeros(1,q); % Lower bounds.
ub=zeros(1,q); % Upper bounds.

% Store initial radial and angular coordinates.
r0=r;
theta0=theta;

% Jitter the initial parameter vector to avoid local minima and ensure feasibility.
[r_i,theta_i]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home);

% Initial parameter vector: [phi0_0, r_i, theta_i].
p0=[phi0_0,r_i, theta_i];


%% Set lower bounds on parameters:
% - Lower bound on the first heading vector offset is 

lb(1)=min([wrapTo2Pi((pi/2)-(theta0(2))),wrapTo2Pi((1.5*pi)-(theta0(2)))]);

% - Lower bounds for radial coordinates:
% - Anchor radii are bounded below by r_home (except for the home anchor).

lb(2:2+(no_anchors-1))=repmat(r_home,1,no_anchors);
lb(2+no_anchors-1)=0; % Home anchor radius is 0.
lb(2)=0; % Home anchor radius is 0.

% - Lower bounds for angular coordinates: 0.
lb(2+no_anchors:end)=zeros(1,no_anchors);

%% Set upper bounds on parameters:
% - Upper bound on the first heading vector offset:

ub(1)=max([wrapTo2Pi((pi/2)-(theta0(2))),wrapTo2Pi((1.5*pi)-(theta0(2)))]); 

% - Upper bounds for radial coordinates: rg_r.
ub(2:2+(no_anchors-1))=repmat(rg_r,1,no_anchors);

% - Upper bounds for angular coordinates: pi.
ub(2+no_anchors:end)=repmat(pi,1,no_anchors);

% - Home anchor radii are 0.
ub(2+no_anchors-1)=0;
ub(2)=0;

 % Initialize the optimizer object.
optimizer_obj=OptimizerClass( );

% Perform constrained optimization using fmincon.
[optimal_para,fval,exitflag,output,lambda,grad,hessian]=fmincon(@(p)optimizer_obj.loss(p,arena,rho,no_anchors,dt)...
    ,p0,[],[],[],[],lb,ub,@(p)non_linear_const_around_ancs(p,no_anchors,tol_radius,r0,theta0,rg_r,rg_th,arena),opts);

% Evaluate the loss function with the optimized parameters.
[~]=optimizer_obj.loss(optimal_para,arena,rho,no_anchors,dt);

end

%%

function [c,ceq]= non_linear_const_around_ancs(p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th,arena)
% My_r_theta_circle_cons - Defines nonlinear constraints for the optimization problem.
%                          Ensures that anchor points stay within a tolerance radius of their initial positions
%                          and respects the boundaries of the arena.
%
% Inputs:
%   p                    - Optimization parameter vector [phi0_0, r1, r2, ..., rn, theta1, theta2, ..., thetan].
%   no_anchors           - Number of anchor points.
%   tol_radius           - Tolerance radius for anchor points (maximum allowed deviation from initial positions).
%   initial_anchors_r    - Initial radial coordinates of anchor points.
%   initial_anchors_theta - Initial angular coordinates of anchor points.
%   rg_r                 - Range of radial values.
%   rg_th                - Range of angular values.
%   arena                - Arena dimensions [x_min, x_max, y_min, y_max].
%
% Outputs:
%   c                    - Nonlinear inequality constraints (must be <= 0).
%   ceq                  - Nonlinear equality constraints (must be == 0). Here, ceq is empty.


%%

% Initialize equality constraints (none in this case).
ceq=[];

% Normalize the radial and angular deviations from the initial anchor positions.
r_norm=(p(2:2+(no_anchors-1))-initial_anchors_r)./rg_r;
th_norm=(p(2+no_anchors:end)-initial_anchors_theta)./rg_th;

% First nonlinear constraint: Ensure anchor points stay within a tolerance radius of their initial positions.
% This is represented as (r_norm)^2 + (th_norm)^2 <= tol_radius^2.
c1= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2);

% Second nonlinear constraint: Ensure anchor points stay within the arena boundaries.
% Define angles corresponding to the arena walls:
% - Right vertical wall (x = arena(2)).
% - Left vertical wall (x = arena(1)).
% - Top wall (y = arena(4)).

 % Compute angles for the right vertical wall.
theta1=[atan2(arena(3),arena(2)),...
    atan2(arena(4),arena(2))];

 % Compute angles for the left vertical wall.
theta3=[atan2(arena(4),arena(1)),...
    atan2(arena(3),arena(1))];

% Compute angles for the top wall.
theta2=[atan2(arena(4),arena(2)),...
    atan2(arena(4),arena(1))];

% Initialize the second constraint vector.
c2=zeros(size(c1));

% Loop through each anchor point and compute its constraint based on its position.
for n=1:no_anchors

    theta_n=p((2+no_anchors-1)+n) ; % Angular coordinate of the nth anchor.
    r_n=p(1+n);  % Radial coordinate of the nth anchor.

    % Check if the anchor point angle lies along the right vertical wall.
    if theta_n>theta1(1) && theta_n<=theta1(2)  
             c2(n)= r_n- (arena(2)./cos(theta_n)); % Constraint for the right wall.
    
    % Check if the anchor point lies along the left vertical wall.         
    elseif theta_n>theta3(1) && theta_n<=theta3(2)  
             c2(n)= r_n- (arena(1)./cos(theta_n)); % Constraint for the left wall.

    % Check if the anchor point lies along the top wall.
    elseif  theta_n>theta2(1) && theta_n<=theta2(2)
             c2(n)= r_n- (arena(4)./sin(theta_n)); % Constraint for the top wall.
    end
    
end

 % Combine the two constraints into a single vector.
 % The optimization requires that c <= 0.
c=max([c1',c2'],[],2);

end

%%

function [r_jitt,theta_jitt]=jitter_initial_para_vector(r0,theta0,tol_radius,rg_r,rg_th,arena,no_anchors,r_home)
% Jitters the initial anchor points to introduce small perturbations.
%                              Ensures that the jittered points stay within a tolerance radius of their initial positions
%                              and respect the arena boundaries.
%
% Inputs:
%   r0              - Initial radial coordinates of anchor points.
%   theta0          - Initial angular coordinates of anchor points.
%   tol_radius      - Tolerance radius for jittering (maximum allowed deviation from initial positions).
%   rg_r            - Range of radial values.
%   rg_th           - Range of angular values.
%   arena           - Arena dimensions [x_min, x_max, y_min, y_max].
%   no_anchors      - Number of anchor points.
%   r_home          - Minimum allowed radial value (home position).
%
% Outputs:
%   r_jitt          - Jittered radial coordinates of anchor points.
%   theta_jitt      - Jittered angular coordinates of anchor points.


%%

% Initialize the jittered radial and angular coordinates.
% The first anchor (home anchor) is not jittered.
r_jitt(1)=r0(1);
theta_jitt(1)=theta0(1);

% Jitter all anchors except the home anchor.
for n=2:no_anchors-1
    
    % Keep jittering until a feasible point is found.

    while(1)
       
        % Add Gaussian noise to the radial and angular coordinates.
        r_jitt(n)=r0(n)+(randn(1)*tol_radius*rg_r);
        theta_jitt(n)=theta0(n)+(randn(1)*tol_radius*rg_th);

        % Ensure the jittered radial coordinate is non-negative.
        if (r_jitt(n)<0)
            r_jitt(n)=0;
        end


        % Ensure the jittered radial coordinate is outside the home region.
        if (r_jitt(n)<=r_home || ((r_jitt(n)-r_home)<=1e-2))
            r_jitt(n)=r_home+(1e-2);
        end

        
        % Ensure the jittered angular coordinate is within (0, pi).
        % and not equal 0 or pi to avoid starting the optimization problem
        % at the angles and radii upper and lower bounds.

        if(theta_jitt(n)>=pi || ((pi-theta_jitt(n))<=1e-2))
            theta_jitt(n)=pi-(1e-2); % Set to just below pi.
        end

        if (theta_jitt(n)<=0 || (theta_jitt(n)<=1e-2))
            theta_jitt(n)=0+(1e-2); % Set to just above 0.
        end


        % Condition 1: Ensure the jittered point stays within 
        % the tolerance radius of the initial point.

        r_norm=(r_jitt(n)-r0(n))./rg_r;
        th_norm=(theta_jitt(n)-theta0(n))./rg_th;
        c1= ( r_norm .^2)+...
        (th_norm.^2)-(tol_radius^2);


        % Condition 2: Ensure the jittered point stays within the arena boundaries.
        % Define angles corresponding to the arena walls:
        % - Right vertical wall (x = arena(2)).
        % - Left vertical wall (x = arena(1)).
        % - Top wall (y = arena(4)).


        % Compute angles for the right vertical wall.
        theta1=[atan2(arena(3),arena(2)),...
            atan2(arena(4),arena(2))];
        
        % left vertical wall
        theta3=[atan2(arena(4),arena(1)),...
            atan2(arena(3),arena(1))];
        
        % top wall
        theta2=[atan2(arena(4),arena(2)),...
            atan2(arena(4),arena(1))];

         % Initialize the second constraint.
         c2 = 0;
        
        % Check if the jittered point is along the right vertical wall. 
        if theta_jitt(n)>theta1(1) && theta_jitt(n)<=theta1(2)  
                 c2= r_jitt(n)- (arena(2)./cos(theta_jitt(n)));

        % along the left vertical wall         
        elseif theta_jitt(n)>theta3(1) && theta_jitt(n)<=theta3(2)  
                 c2= r_jitt(n)- (arena(1)./cos(theta_jitt(n)));

        % lastly, along the top wall 
        elseif  theta_jitt(n)>theta2(1) && theta_jitt(n)<=theta2(2)
                 c2= r_jitt(n)- (arena(4)./sin(theta_jitt(n)));
       end

       % If the jittered point satisfies both conditions, break the loop.
       if((c1<0)&&(c2<0))
           break
       end

    end

end

r_jitt(end+1)=r0(end);
theta_jitt(end+1)=theta0(end);

end

