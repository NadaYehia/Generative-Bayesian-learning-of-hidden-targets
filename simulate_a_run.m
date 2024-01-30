function [target_hit,hit_time]=simulate_a_run(pos_x,pos_y,arena,target_num,target,target2,target_hit,hit_time)

    for t=2:size(pos_x,2)
    
        if pos_x(t)<arena.dims(1)
            pos_x(t)=arena.dims(1);                
        elseif pos_x(t)>arena.dims(2)
            pos_x(t)=arena.dims(2);
        end
        
        if pos_y(t)<arena.dims(3)
            pos_y(t)=arena.dims(3);                
        elseif pos_y(t)>arena.dims(4)
            pos_y(t)=arena.dims(4);
        end
        
    
        if (target_num==1)
           curr_target = target;
        else
            curr_target = target2;            
        end
        
        % the agent hit the target box at any point inside it.    
        if pos_y(t)>curr_target.cntr(2)-curr_target.width(2) & pos_y(t)<curr_target.cntr(2)+curr_target.width(2) & pos_x(t)>curr_target.cntr(1)-curr_target.width(1) & pos_x(t)<curr_target.cntr(1)+curr_target.width(1)
            target_hit= 1;
            if hit_time==1
                hit_time= t;
            end
            break; 
        end

    end

end