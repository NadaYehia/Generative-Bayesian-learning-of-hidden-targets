function [r,theta,x_op,y_op]=trajectory_planner(r_anchors,theta_anchors,env,clearnce,Os,Rs,...
ka,w20,tol_radius)

% INPUTS: {r,theta} anchors: r_anchors, theta_anchors
%         Environment boundaries: env Object
%         Initial offset of the heading vector from home to first anchor:
%         clearnce (in radians).
%         Vectors of radii and angles in the posterior space: Rs, Os
%         
%        Traveling time to distance between anchors scale factor: ka
%        Penalty term weight for abrupt changes in the trajectory heading 
%                                                         angles: w20    
%        Allowed error radius around input anchors in the optimization of
%             [path length + smoothness]: tol_radius
%
% OUTPUTS: {r,theta} polar coordinates of all the points on the planned 
%                    path
%          {x_op,y_op} output x and y coordinates of the planned path 
%                     
%
% EXAMPLE: 
% env=environment;
% env.arena_dimensions= [-375,375,0,750];
% [r_path,theta_,x_op,y_op]= trajectory_planner([0 300 300 0],...
%              [0 pi/2 0.2 0],env,0.2,Os,Rs,0.01,1000,0.03);

%          figure,scatter(x_op,y_op,30,[1:numel(x_op)])  
                                 % plots the path weaved through input anchors 
                                 % optimized for path length and smooth-
                                 % ness.      
%
%% Initalizations:

exitflag=0;
dt=0.01; % time discretization for simulating a path between 2 anchors.
x_=[];
y_=[];
arena=env.arena_dimensions;
rg_r=abs(Rs(end)-Rs(1));
rg_th=abs(Os(end)-Os(1));
tto=0;
MaxTrials=9; % Maximum number of optimization runs
  
kd=[clearnce, zeros(1,numel(r_anchors)-2)]; % (n-1) row vector of the initial 
                                            % heading angles of the (n-1) 
                                            % segments connecting between 
                                            % every anchor pair in {n}anchors.



theta0=theta_anchors+(pi/2);
r0=r_anchors;

% angles of the (n-1) vectors connecting every anchor pair in{n anchors}:
for n=2:numel(r0)

    heading_offsets(n-1)= theta0(n-1)+atan2( r0(n)*sin(theta0(n)-theta0(n-1)),...
                                    r0(n)*cos(theta0(n)-theta0(n-1)) -r0(n-1)  );
    
end

for n=2:size(r0,2)-1
 
    % ensure continuity by setting 
    % the offset of the current heading vector kd(n) at time t equal to the 
    % the current heading offset - heading angle at t-1.
     
    last_heading_previous_seg=wrapToPi(heading_offsets(n-1)+kd(n-1)); 
    kd(n)=wrapToPi(heading_offsets(n)- last_heading_previous_seg);
 
end

%% Optimize [anchor distance, anchor heading, initial heading angle:
% r,theta,kd] for every anchor to minimize total path length and sum (angul-
% ar changes at anchor points)

MFE=30000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,"EnableFeasibilityMode",true);
w2=w20;
ri=r0;
thetai=theta0;

while(exitflag<=0)

    [optimal_para,fval,exitflag,output,~,~,~]=...
        optimize_path_length_and_smoothness(ri,thetai,kd,arena,ka,w2,tol_radius,rg_r,rg_th,opts,dt);
    
    if(exitflag<=0 )
        %noise to the starting point, very small values
        ri=r0+0.0001*randn(1);
        thetai=theta0+0.0001*randn(1);
    end

    tto=tto+1;
     
    if(tto>MaxTrials) % if failed to find feasible solution
        % use input {r,theta anchors} in the next step.  
        optimal_para=[];
        optimal_para=[kd,r0,theta0];
        break;
    end
      
end

% ERROR CHCK for optimized parameters output
if(any(isnan(optimal_para)) )

    r_anchors
    theta_anchors
    error('invalid solution, optimized para has Nans')

end



%% Plan {x,y} points along the path between optimized anchors 
ancs_no=numel(r0);
k_d_optimal=optimal_para(1:numel(r0)-1);
r_optimal=optimal_para(ancs_no:ancs_no+ancs_no-1);
theta_optimal=optimal_para(ancs_no+ancs_no:end);
x_op=[];
y_op=[];

for n=1:size(r_optimal,2)-1
 
    vx=(-r_optimal(n)*cos(theta_optimal(n))) +(r_optimal(n+1)*cos(theta_optimal(n+1)));
    vy=(-r_optimal(n)*sin(theta_optimal(n))) +(r_optimal(n+1)*sin(theta_optimal(n+1)));

    heading_offset= (atan2(vy,vx ));

    if(n~=1)
        
        k_d_optimal(n)=wrapToPi(k_d_optimal(n));
        tol=abs(k_d_optimal(n));
        k=sign(k_d_optimal(n));
        epsi=((tol-pi/2));
    else
        
        k_d_optimal(1)=wrapToPi(k_d_optimal(1));
        k=sign(k_d_optimal(n));
        tol=abs(k_d_optimal(n));
        epsi=((tol-pi/2));
    end
    
   
    eucl_dist=sqrt(vx^2 +vy^2);   

    % use the euclidean dist between a pair of anchors and the initial
    % heading offset to find the speed function parameter needed to
    % connect these two anchors.
    vmax_n=eucl_dist*((4*pi)+(4*epsi));
    vmax_d= (pi*ka)*(sinc(epsi/pi));
    vmax= sqrt(vmax_n/vmax_d);

    % scale the total traveling time from anchor i to anchor j
    % by the speed function control parameter
    T=ka*vmax;

    if(~isreal(T))
        error('Time cant be complex, sinc function is outside pi and -pi');
    end
    
    % compute the speed and heading angle time functions given the
    % trajectory model: speed= A*sin(wt), heading= (4*kd/T)*t +
    % (offset-kd).
    w=(2*pi)/(T);
    t1=[0:dt:T/2];
    speed= [sin(w.*t1)];
    speed= (vmax).*speed;
    
    heading= ( ((4*k*tol)/T) .*t1)+( (heading_offset) -(k*tol));
    heading=wrapToPi(heading);
   
    % calculate the x&y points of a trajectory segment
    pos_x=zeros(1,size(heading,2));
    pos_y=zeros(1,size(heading,2));
    dx= zeros(1,size(heading,2));
    dy=zeros(1,size(heading,2));
    dx(1)=r_optimal(n)*cos(theta_optimal(n));
    dy(1)=r_optimal(n)*sin(theta_optimal(n));
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


    x_op=[x_op, pos_x(1:end)];
    y_op=[y_op, pos_y(1:end)];

    pos_y=[];
    pos_x=[];


end

%% Compute {r,omegas} from the optimized path {x,y} points
 [r,theta]= convert_xy_r_angle(x_op,y_op);
  


end