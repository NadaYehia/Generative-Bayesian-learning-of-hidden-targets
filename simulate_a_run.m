function [target_hit,hit_time]=simulate_a_run(pos_x,pos_y,target_num,env,hit_time)

 % Simulates a run to determine if the agent hits a target and records the time of the hit.
    % Inputs:
    %   pos_x: X-coordinates of the agent's position over time.
    %   pos_y: Y-coordinates of the agent's position over time.
    %   target_num: The index of the target to check for a hit.
    %   env: Environment structure containing target centers and dimensions.
    %   hit_time: Initial hit time (used to store the first hit time).
    % Outputs:
    %   target_hit: 1 if the target is hit, 0 otherwise.
    %   hit_time: Time step when the target is first hit (if hit_time input is 1).


%%

% Loop through each time step    
for t=1:size(pos_x,2)
    
    % Get the center coordinates of the current target
    curr_target_centers=env.targets_centers(target_num,:);
    % Get the dimensions (width and height) of the current target
    curr_target_dims=env.targets_dimensions(target_num,:);
    
    % Check if the agent's position is inside the target box
    if pos_y(t)>curr_target_centers(2)-(curr_target_dims(2)/2) ... % Check lower Y bound
       & pos_y(t)<curr_target_centers(2)+(curr_target_dims(2)/2) & ... % Check upper Y bound
         pos_x(t)>curr_target_centers(1)-(curr_target_dims(1)/2) & ... % Check lower X bound
         pos_x(t)<curr_target_centers(1)+(curr_target_dims(1)/2) % Check upper X bound

        % Target is hit
        target_hit= 1;

        % If hit_time is 1 (initial value), record the current time step
        if hit_time==1
            hit_time= t;
        end

        % Exit the loop since the target has been hit
        break; 

    else 
        % Target is not hit at this time step
        target_hit=0;

    end

end

end