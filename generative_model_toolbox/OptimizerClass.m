classdef OptimizerClass< handle
    % OptimizerClass - A class for optimizing trajectories through anchor points.
    %                   The class computes the trajectory cost and generates optimized paths.
    %
    % Properties:
    %   optimized_x - X-coordinates of the optimized trajectory.
    %   optimized_y - Y-coordinates of the optimized trajectory.
    %   param       - Optimization parameters (radii, angles, and heading offsets).

    properties
        optimized_x=[]; % Stores the optimized x-coordinates of the trajectory.
        optimized_y=[]; % Stores the optimized y-coordinates of the trajectory.
        param=[]; % Stores the optimization parameters.
    end

%%

    methods

        function total_cost=loss(obj,p,arena,rho,no_anchors,dt)

        % loss - Computes the total cost of the trajectory and generates the optimized path.
        %
        % Inputs:
        %   p          - Optimization parameter vector [phi0_0, r1, r2, ..., rn, theta1, theta2, ..., thetan].
        %   arena      - Arena dimensions [x_min, x_max, y_min, y_max].
        %   rho        - Weighting factor for the optimization objective.
        %   no_anchors - Number of anchor points.
        %   dt         - Time step for trajectory generation.
        %
        % Output:
        %   total_cost - Total cost of the trajectory (sum of path lengths).

        % Initialize variables.
        x_op=[]; % X-coordinates of the optimized trajectory.
        y_op=[]; % Y-coordinates of the optimized trajectory.
        speed_conca=[]; % Concatenated speed values for the trajectory.
        heading_conca=[]; % Concatenated heading values for the trajectory.

        % Extract radial and angular coordinates from the parameter vector.
        r=p(2:2+no_anchors-1);
        theta=p(2+no_anchors:end);

        % Initialize heading offsets.
        phi0_i=zeros(1,no_anchors-1); % Heading offsets between segments.

        phi0_i(1)=p(1); % Initial heading offset.

        %%
        % Compute heading vectors between consecutive anchor points.
        for n=1:numel(r)-1

           heading_vectors(n)= atan2((r(n+1)*sin(theta(n+1)))-(r(n)*sin(theta(n))),...
               (r(n+1)*cos(theta(n+1)))-(r(n)*cos(theta(n))));

        end
        heading_vectors=wrapTo2Pi(heading_vectors);
        
        % Generate trajectory segments between anchor points.
        for n=1:size(r,2)-1
        
            % Compute the change in x and y between consecutive anchor points.
            vx=(-r(n)*cos(theta(n))) +(r(n+1)*cos(theta(n+1)));
            vy=(-r(n)*sin(theta(n))) +(r(n+1)*sin(theta(n+1)));
            
            % Ensure continuity of heading offsets between segments.
            if(n~=1)
               
%                 last_heading_previous_seg=wrapToPi(+(pi/2)-phi0_i(n-1)); 
                phi0_i(n)=wrapTo2Pi((pi-phi0_i(n-1))-(heading_vectors(n)-heading_vectors(n-1)));
                
               
            else
    
                 % Wrap the initial heading offset to the range [-pi, pi].
                phi0_i(1)=wrapTo2Pi(phi0_i(1));
                
            end
            
            % Compute the Euclidean distance between consecutive anchor points.
            eucl_dist(n)=sqrt(vx^2 +vy^2);
            
            % Compute the time duration for the current segment.
            T=eucl_dist(n)/rho;
            
            % Compute the maximum speed for the current segment.
            vmax_n=2*rho* (((3*pi)-(2*phi0_i(n)))/T) * ((pi+(2*phi0_i(n)))/T);
            vmax_d= (((2*pi)/T)^2)*(sinc( (pi-(2*phi0_i(n)))/(2*pi) ));
            vmax= (vmax_n/vmax_d);
            
            
             % Check for invalid maximum speed.
            if( (vmax==0) || (sign(vmax)<0) )
                error('velocity is 0 or negative: check your calculations');
            end
            
             % Generate speed and heading profiles for the current segment.
            
            t1=0:dt:T; % Time vector.
            speed=(1-cos((2*pi*t1)/T));  % Speed profile.
            speed=(vmax/2).*speed;  % Scale speed by maximum speed.

            heading=((pi-(2*phi0_i(n)))/T).*t1 + heading_vectors(n)-(pi/2)+ phi0_i(n); % Heading profile.
            heading=wrapToPi(heading); % Wrap heading to the range [-pi, pi].
            
            % Compute the x and y coordinates of the trajectory segment.

            pos_x=zeros(1,size(heading,2)); % Initialize x-coordinates
            pos_y=zeros(1,size(heading,2)); % Initialize y-coordinates
            dx= zeros(1,size(heading,2)); % Initialize x-displacements
            dy=zeros(1,size(heading,2));  % Initialize y-displacements

            dx(1)=r(n)*cos(theta(n)); % Initial x-displacement.
            dy(1)=r(n)*sin(theta(n)); % Initial y-displacement.

            % integrate x and y displacements to find x&y coordinates of
            % the segment points.

            [temp_dx,temp_dy] = pol2cart(heading(2:end),speed(2:end).*dt);
            dx(2:end)=temp_dx;
            dy(2:end)=temp_dy;
            pos_x=cumsum(dx);
            pos_y=cumsum(dy);
            
            % Compute the path length of the current segment
            Pl(n)=( sum(vecnorm([diff(pos_x)' diff(pos_y)'],2,2)) );
            
            % Confine the trajectory segment to the arena boundaries.
            pos_x( find(pos_x>arena(2)) )=arena(2); 
            pos_x( find(pos_x<arena(1)) )=arena(1);
            pos_y( find(pos_y>arena(4)) )=arena(4); 
            pos_y( find(pos_y<arena(3)) )=arena(3); 
                        
            % Concatenate the current segment with the overall trajectory.
            x_op=[x_op, pos_x(1:end)];
            y_op=[y_op, pos_y(1:end)];
            speed_conca=[speed_conca,speed];
            heading_conca=[heading_conca, heading];
            
            % Clear temporary variables.
            pos_y=[];
            pos_x=[];
        
        end

        % Store the optimized trajectory and parameters.
        obj.optimized_x=x_op;
        obj.optimized_y=y_op;
        obj.param=p;

        % Compute the total cost as the sum of path lengths.
        total_cost= (sum(Pl));
       
        end


    end
 
end

