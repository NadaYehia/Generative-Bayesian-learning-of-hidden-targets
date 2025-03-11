function [r,theta,x_op,y_op,exitflag]=trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,...
rho,tol_radius,r_home,dt)

% trajectory_planner - Plans an optimal trajectory through a set of anchor points.
%                      The trajectory minimizes path length and angular changes while respecting constraints.
%
% Inputs:
%   r_anchors      - Radial coordinates of anchor points.
%   theta_anchors  - Angular coordinates of anchor points.
%   env            - Environment structure containing arena dimensions.
%   Ths            - Vector of possible angular values.
%   Rs             - Vector of possible radial values.
%   rho            - Weighting factor for the optimization objective.
%   tol_radius     - Tolerance radius for constraints.
%   r_home         - Minimum allowed radial value (home position).
%   dt             - Time step for trajectory generation.
%
% Outputs:
%   r              - Radial coordinates of the optimized trajectory.
%   theta          - Angular coordinates of the optimized trajectory.
%   x_op           - X-coordinates of the optimized trajectory.
%   y_op           - Y-coordinates of the optimized trajectory.
%   exitflag       - Exit flag from the optimization process (indicates success or failure).



%%

% Initializations:
arena=env.arena_dimensions; % Extract arena dimensions from the environment.

rg_r=abs(Rs(end)-Rs(1)); % Compute the range of radial values.

rg_th=abs(Ths(end)-Ths(1)); % Compute the range of angular values.
eps=1e-2; % Small epsilon value to avoid theta and radii values starting at their upper 
          % and lower bounds in the optimization problem.

trials_to_optimize=0; % Counter for optimization trials.

% Adjust initial anchor points angles to lie between 0 and pi.
theta0=theta_anchors+(pi/2); % Shift angular coordinates by pi/2.
r0=r_anchors; % Use radial coordinates as-is.
r0(r0==0)=eps; % Replace zero radial values with epsilon.
theta0(theta0==pi)=pi-(eps); % Adjust angular values close to pi.
theta0(theta0==0)=0+(eps); % Adjust angular values close to 0.

% initialize the initial heading angle offset (phi0_0).
if(rand(1)>0.5)
    phi0_0=wrapTo2Pi(-(theta0(2)-(pi/2))); % (clockwise direction of rotation)
                                      % and offset angle resulting in phi(0)~ 0
else
    phi0_0=wrapTo2Pi(pi-(theta0(2)-(pi/2))); % anticlockwise direction of rotation and offset angle resulting 
                                          % in phi(0)~ pi
end

%%

% Set optimization options for fmincon.

MFE=10000; % Maximum function evaluations.
Itrs=1000; % Maximum iterations.
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,'Display','notify',...
    'Algorithm','interior-point');
    
% Perform the initial optimization.
[optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
    optimize_path_length(r0,theta0,phi0_0,arena,rho,tol_radius,rg_r,rg_th,opts,dt,r_home);
  
% Retry optimization if the initial attempt fails (up to 2 retries).
while (exitflag<=0 && (trials_to_optimize<2))
       
        [optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
         optimize_path_length(r0,theta0,phi0_0,arena,rho,tol_radius,rg_r,rg_th,opts,dt,r_home);

        trials_to_optimize=trials_to_optimize+1;
end

% Use the best feasible solution.
optimal_para=output.bestfeasible.x;

% Evaluate the loss function with the optimized parameters.
[~]=optimizer_obj.loss(optimal_para,arena,rho,numel(r0),dt);


% Throw an error if no feasible solution is found
if(exitflag==-2 && isempty(output.bestfeasible))
                                                  
    error('infeasbility');

end

% Extract the optimized trajectory coordinates.
x_op=optimizer_obj.optimized_x;
y_op=optimizer_obj.optimized_y;


 % Convert the optimized Cartesian coordinates (x, y) to polar coordinates (r, theta).
[r,theta]= convert_xy_r_angle(x_op,y_op);
  

end