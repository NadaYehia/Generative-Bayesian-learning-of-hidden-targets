function [mu, omega]=convert_xy_velo_angle(x_op,y_op)
 omega=[];
 mu=[];
 T=1000;
% scale factor of the area under the speed curve.
w=(2*pi)/(T);
t1=[0:(T/2)];
speed= [sin(w.*t1)]; 

k=sum(speed);
arena.dims = [-400 400 0 600];

% figure;
for i=1:numel(x_op)

    heading_offset= atan2(y_op(i),x_op(i)); % this goes from -pi/2 to pi/2, as omega in our model.

    omega(i)=heading_offset-(pi/2);

    tol=0.5;
    m=(4*tol)/T;

    r1=k*y_op(i)* ( (pi^2)-(4* (tol^2)) );
    q1= pi*T*(cos(tol)*sin(heading_offset));
    mu_y= r1/q1;

    r=k*x_op(i)* ( (pi^2)-(4* (tol^2)) );
    q= pi*T*(cos(tol)*cos(heading_offset));
    mu_x= r/q;

    mu(i)=mu_x;

    if (isinf(abs(mu(i))))
        mu(i)=mu_y;
    
    end
% debug: plot this half circle with mu and omega calc above.

    w=(2*pi)/(T);
    t1=[0:(T/2)];
    speed= [sin(w.*t1)];
    speed=speed./sum(speed);

    speed= (mu(i)).*speed;  
    
    heading= (m.*t1)+(heading_offset-tol);   
    
    pos_x(1)=0;
    pos_y(1)=0;
    
    for t=2:T/2
    [dx,dy] = pol2cart(heading(t),speed(t));
     pos_x(t)=pos_x(t-1)+dx;

        if pos_x(t)<arena.dims(1)
            pos_x(t)=arena.dims(1);                
        elseif pos_x(t)>arena.dims(2)
            pos_x(t)=arena.dims(2);
        end
% 
         pos_y(t)=pos_y(t-1)+dy;

        if pos_y(t)<arena.dims(3)
            pos_y(t)=arena.dims(3);                
        elseif pos_y(t)>arena.dims(4)
            pos_y(t)=arena.dims(4);
        end
    
    end

%     hold on, scatter(x_op(i),y_op(i),30,'green','filled');
%     hold on, plot(pos_x,pos_y);
%     clf;
    pos_y=[];
    pos_x=[];

end


end