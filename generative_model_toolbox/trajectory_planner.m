function [r,theta,x_op,y_op,exitflag]=trajectory_planner(r_anchors,theta_anchors,env,Ths,Rs,...
rho,tol_radius,r_home,dt,eps,max_opti_trials)
% trajectory_planner - Plans an optimal trajectory through a set of anchor points.
%                      The trajectory minimizes path length and angular changes while respecting constraints.
%
% Inputs:
%   r_anchors      - Radial coordinates of anchor points.
%   theta_anchors  - Angular coordinates of anchor points.
%   env            - Environment structure containing arena dimensions.
%   Ths            - Vector of possible angular values.
%   Rs             - Vector of possible radial values.
%   rho            - scaling factor of the timing function with distance. 
%   tol_radius     - Tolerance radius for constraints.
%   r_home         - Minimum allowed radial value (home position).
%   dt             - Time step for trajectory generation.
%   eps            -Small epsilon value to avoid numerical issues with
%                   thetas or phi0_0 starting the optimization at their upper or lower bounds.
%   max_opti_trials - number of optimization retries if optimization fails.
%
% Outputs:
%   r              - Radial coordinates of the optimized trajectory.
%   theta          - Angular coordinates of the optimized trajectory.
%   x_op           - X-coordinates of the optimized trajectory.
%   y_op           - Y-coordinates of the optimized trajectory.
%   exitflag       - Exit flag from the optimization process (indicates success or failure).



%%
% Initalizations:

arena=env.arena_dimensions; % Dimensions of the arena.
rg_r=abs(Rs(end)-Rs(1)); % Range of radial distances.
rg_th=abs(Ths(end)-Ths(1)); % Range of angles.
trials_to_optimize=0; % Counter for optimization trials.

% Initial conditions for radius and angle:
theta0=theta_anchors+(pi/2); % Adjust initial angles by 90 degrees.
r0=r_anchors; % Initial radial distances.
r0(r0==0)=eps; % Replace zero radii with epsilon to avoid starting the optimization problem
               % with the radii values at their lower boundary in the optimization problem.
theta0(theta0==pi)=pi-(eps); % Adjust angles close to pi.
theta0(theta0==0)=0+(eps);   % Adjust angles close to zero.

% Calculate the heading vectors between consecutive anchor points:
for n=2:numel(r0)

    heading_vectors(n-1)= theta0(n-1)+atan2( r0(n)*sin(theta0(n)-theta0(n-1)),...
                                    r0(n)*cos(theta0(n)-theta0(n-1)) -r0(n-1)  );
    
end

% initialize phi0_0 (randomly in clockwise or anti-clockwise direction
% off the first heading vector:
if(rand(1)>0.5)
    phi0_0=heading_vectors(1)-eps;
else
    phi0_0=(-(pi-heading_vectors(1)))+eps;
end

%%
% Optimization settings:
MFE=10000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,'Display','notify',...
    'Algorithm','interior-point'); % Optimization options.

% Optimize the path length and smoothness:
[optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
    optimize_path_length(r0,theta0,phi0_0,arena,rho,tol_radius,rg_r,rg_th,opts,dt,r_home);

% Retry optimization if it fails (up to 'max_opti_trials' trials):
while (exitflag<=0 && (trials_to_optimize<max_opti_trials))
       
        [optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
         optimize_path_length(r0,theta0,phi0_0,arena,rho,tol_radius,rg_r,rg_th,opts,dt,r_home);

        trials_to_optimize=trials_to_optimize+1;
end

% Use the best feasible solution if available:
optimal_para=output.bestfeasible.x;
[~]=optimizer_obj.loss(optimal_para,arena,rho,numel(r0),dt);

% Handle infeasible solutions:
if(exitflag==-2 && isempty(output.bestfeasible)) %  If the solution is infeasible 
                                                 % and no feasible solution was found.
        error(['infeasbility:...' ...
            'the initial point is out of the arena bounds or does not respect the non linear ' ...
            'constraints. check your initial parameter vector for feasibility.']) % Throw an error.
end

% Extract optimized path coordinates:
x_op=optimizer_obj.optimized_x;
y_op=optimizer_obj.optimized_y;


 % Convert optimized path coordinates to polar coordinates:
 [r,theta]= convert_xy_r_angle(x_op,y_op);
  

end