
classdef OptimizerClass_Pl< handle
    properties
        optimized_x=[];
        optimized_y=[];
        param=[];
    end

    methods

        function total_cost=my_loss(obj,p,arena,ka,no_anchors,w1,dt)
        x_op=[];
        y_op=[];
        speed_conca=[];
        heading_conca=[];
        
        k_d0=p(1);
        r=p(2:2+no_anchors-1);
        theta=p(2+no_anchors:end);
        k_d=zeros(1,no_anchors-1);
        k_d(1)=k_d0;

        for n=2:numel(r)

           heading_offset(n-1)= theta(n-1)+atan2( r(n)*sin(theta(n)-theta(n-1)),...
                                    r(n)*cos(theta(n)-theta(n-1)) -r(n-1)  );

        end
        


        for n=1:size(r,2)-1
        
        vx=(-r(n)*cos(theta(n))) +(r(n+1)*cos(theta(n+1)));
        vy=(-r(n)*sin(theta(n))) +(r(n+1)*sin(theta(n+1)));
        
        if(n~=1)
           
       
            % ensure continuity by setting 
            % the offset of the current heading vector kd(n) at time t equal to the 
            % the current heading offset - heading angle at t-1.
             
            last_heading_previous_seg=wrapToPi(heading_offset(n-1)+k_d(n-1)); 
            k_d(n)=wrapToPi(heading_offset(n)- last_heading_previous_seg);
            
            k_d(n)=wrapToPi(k_d(n));
            tol=abs(k_d(n));
            k=sign(k_d(n));
            epsi=((tol-pi/2));
             
        else
            
            k_d(1)=wrapToPi(k_d(1));
            k=sign(k_d(1));
            tol=abs(k_d(1));
            epsi=((tol-pi/2));
        end
        
        
        eucl_dist(n)=sqrt(vx^2 +vy^2);
        vmax_n=ka*((4*pi)+(4*epsi));
        vmax_d= (pi)*(sinc(epsi/pi));
        vmax= (vmax_n/vmax_d);
        
        T=eucl_dist(n)/ka;
        
            if(~isreal(T))
                error('Time cant be complex, sinc function is outside pi and -pi');
            end
        
        % use the functional form to produce x,y points in space
        w=(2*pi)/(T);
        t1=[0:dt:T/2];
        
        speed= [sin(w.*t1)];
        speed= (vmax).*speed;
        heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset(n)) -(k*tol));
        heading=wrapToPi(heading);
        
        % calculate the x&y points of a trajectory segment
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
        

        % confine the trajectory segment to the arena enclosure
        pos_x( find(pos_x>arena(2)) )=arena(2); 
        pos_x( find(pos_x<arena(1)) )=arena(1);
        pos_y( find(pos_y>arena(4)) )=arena(4); 
        pos_y( find(pos_y<arena(3)) )=arena(3); 
        
        Pl(n)=( sum(vecnorm([diff(pos_x)' diff(pos_y)'],2,2)) );
        
        x_op=[x_op, pos_x(1:end)];
        y_op=[y_op, pos_y(1:end)];
        speed_conca=[speed_conca,speed];
        heading_conca=[heading_conca, heading];
        
        pos_y=[];
        pos_x=[];
        
        end
        obj.optimized_x=x_op;
        obj.optimized_y=y_op;
        obj.param=p;
        total_cost= (w1*sum(Pl));
       
        end


    
    end
 


end

