function [target_hit,hit_time]=simulate_a_run(pos_x,pos_y,target_num,env,hit_time)

    for t=2:size(pos_x,2)
    
        curr_target_centers=env.targets_centers(target_num,:);
        curr_target_dims=env.targets_dimensions(target_num,:);
        
        % the agent hit the target box at any point inside it.    
        if pos_y(t)>curr_target_centers(2)-curr_target_dims(2) ...
           & pos_y(t)<curr_target_centers(2)+curr_target_dims(2) & ... 
             pos_x(t)>curr_target_centers(1)-curr_target_dims(1) & ...
             pos_x(t)<curr_target_centers(1)+curr_target_dims(1)

            target_hit= 1;
            if hit_time==1
                hit_time= t;
            end
            break; 

        else 
            target_hit=0;

        end

    end

end