
classdef OptimizerClass< handle
    % OptimizerClass is a handle class that optimizes a trajectory path
    % based on given anchor points and constraints. It calculates the
    % optimized path and its associated cost.
    properties
        optimized_x=[]; % Stores the optimized x-coordinates of the trajectory.
        optimized_y=[]; % Stores the optimized y-coordinates of the trajectory.
        param=[];       % Stores the optimized parameters (radii, angles and phi0_0).
    end

    methods

        function total_cost=loss(obj,p,arena,rho,no_anchors,dt)
            % loss calculates the total cost of the trajectory based on the
            % given parameters and constraints.
            %
            % Inputs:
            %   p: Parameter vector containing [phi0_0, r, theta].
            %   arena: Dimensions of the arena [x_min, x_max, y_min, y_max].
            %   rho: Scaling factor for speed.
            %   no_anchors: Number of anchor points.
            %   dt: Time step for trajectory simulation.
            %
            % Outputs:
            %   total_cost: Total cost of the trajectory (sum of segment lengths).
            
            %%
            % Initialize variables:
            x_op=[]; % Optimized x-coordinates.
            y_op=[]; % Optimized y-coordinates.
            speed_conca=[]; % Concatenated speed values.
            heading_conca=[]; % Concatenated heading values.
            r=p(2:2+no_anchors-1); % Extract radii from the parameter vector.
            theta=p(2+no_anchors:end); % Extract angles from the parameter vector.
            phi0_i=zeros(1,no_anchors-1); % Initialize heading offsets.
            phi0_i(1)=p(1); % Set the first heading offset.
    
            % Calculate heading vectors between consecutive anchor points:
            for n=2:numel(r)
    
               heading_vectors(n-1)= theta(n-1)+atan2( r(n)*sin(theta(n)-theta(n-1)),...
                                        r(n)*cos(theta(n)-theta(n-1)) -r(n-1)  );
    
            end
            


            % Iterate over each segment of the trajectory: 
            for n=1:size(r,2)-1
            % Calculate the vector between consecutive anchor points:
            vx=(-r(n)*cos(theta(n))) +(r(n+1)*cos(theta(n+1)));
            vy=(-r(n)*sin(theta(n))) +(r(n+1)*sin(theta(n+1)));
            
            % Ensure continuity of heading angles between segments:
            if(n~=1)
                % Calculate the heading offset for the current segment:
                last_heading_previous_seg=wrapToPi(heading_vectors(n-1)+phi0_i(n-1)); 
                phi0_i(n)=wrapToPi(heading_vectors(n)- last_heading_previous_seg);
                
                phi0_i(n)=wrapToPi(phi0_i(n));
                abs_phi0_i=abs(phi0_i(n));
                dir_rotation=sign(phi0_i(n));
                epsi=((abs_phi0_i-pi/2));
                 
            else
                % Handle the first segment separately:
                phi0_i(1)=wrapToPi(phi0_i(1));
                dir_rotation=sign(phi0_i(1));
                abs_phi0_i=abs(phi0_i(1));
                epsi=((abs_phi0_i-pi/2));
            end
            
            % Calculate Euclidean distance between anchor points:
            eucl_dist(n)=sqrt(vx^2 +vy^2);

            % Calculate maximum speed for the segment:scale factor for the
            % sinusoidal speed function.
            vmax_n=rho*((4*pi)+(4*epsi));
            vmax_d= (pi)*(sinc(epsi/pi));
            vmax= (vmax_n/vmax_d);
            
            % Calculate time duration to execute the segment:
            T=eucl_dist(n)/rho;
            
            % error check
            if(~isreal(T))
                error('Time cant be complex, sinc function is outside pi and -pi');
            end
            
            % Generate speed and heading profiles for the segment:
            w=(2*pi)/(T);
            t1=[0:dt:T/2];
            
            speed= [sin(w.*t1)];
            speed= (vmax).*speed;
            heading= ( ((4*dir_rotation*abs_phi0_i)/T) .*t1)+( (heading_vectors(n)) -(dir_rotation*abs_phi0_i));
            heading=wrapToPi(heading);
            
            % Calculate the x and y coordinates of the trajectory segment:
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
            
            % Calculate the length of the segment:
            Pl(n)=( sum(vecnorm([diff(pos_x)' diff(pos_y)'],2,2)) );
            
            % Confine the segment to the arena boundaries:
            pos_x( find(pos_x>arena(2)) )=arena(2); 
            pos_x( find(pos_x<arena(1)) )=arena(1);
            pos_y( find(pos_y>arena(4)) )=arena(4); 
            pos_y( find(pos_y<arena(3)) )=arena(3); 
            
            % Concatenate the segment's x, y, speed, and heading values:
            x_op=[x_op, pos_x(1:end)];
            y_op=[y_op, pos_y(1:end)];
            speed_conca=[speed_conca,speed];
            heading_conca=[heading_conca, heading];
            
            % Clear temporary variables:
            pos_y=[];
            pos_x=[];
            
            end

            % Store the optimized trajectory and parameters:
            obj.optimized_x=x_op;
            obj.optimized_y=y_op;
            obj.param=p;

            % Calculate the total cost (sum of segment lengths):
            total_cost= (sum(Pl));
           
        end


    
    end
 


end

