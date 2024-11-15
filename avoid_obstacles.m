function [pos_xc,pos_yc]=avoid_obstacles(pos_x,pos_y,env)

%Obstacle: obstacle center, w, h
eps=env.obstacle(6);

obstacle_=env.setup_obstacle_coord;



i=2;
pos_xc(1:2)=pos_x(1:2);
pos_yc(1:2)=pos_y(1:2);

t=i;
tc=0;
while(t<numel(pos_x)-1)

   
    On_x=(pos_xc(t+tc)> min(obstacle_.x)) & (pos_xc(t+tc)<max(obstacle_.x));
    On_y=(pos_yc(t+tc)> min(obstacle_.y)) & (pos_yc(t+tc)<max(obstacle_.y));
    
    
    if (On_x & On_y)
       
        if((pos_y(t)-pos_y(t-1)) >0)  %approaching obstacle from below
            y_wall= min(obstacle_.y);
            

        else
            
            if((pos_y(t)-pos_y(t-1)) <0)
                y_wall= max(obstacle_.y);
                
            end

        end
        vx=sign(pos_x(t)-pos_x(t-1));
        while( 1)
        %follow local rule: 
        % walk along the wall side till the end then resume the path 
        
        pos_yc(tc+t)=y_wall;
        pos_xc(tc+t)= pos_xc(t+tc-1)+(vx*1);
       
        if(pos_xc(tc+t)> max(obstacle_.x)+eps || pos_xc(tc+t)<min(obstacle_.x)-eps)
            break;
        end
        tc=tc+1;
        end

        

    
    else
        
        pos_xc(tc+t+1)=pos_xc(tc+t)+(pos_x(t+1)-pos_x(t));
        pos_yc(tc+t+1)=pos_yc(tc+t)+((pos_y(t+1)-pos_y(t)));
        t=t+1;

    end

   


end



end