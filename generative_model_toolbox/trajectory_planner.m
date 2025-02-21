function [r,theta,x_op,y_op,exitflag]=trajectory_planner(r_anchors,theta_anchors,env,Os,Rs,...
ka,w1,tol_radius,r_home)


% Initalizations:
dt=0.01; % time discretization for simulating a path between 2 anchors.
arena=env.arena_dimensions;
rg_r=abs(Rs(end)-Rs(1));
rg_th=abs(Os(end)-Os(1));
eps=1e-2;
trials_to_optimize=0;

theta0=theta_anchors+(pi/2);
r0=r_anchors;
r0(r0==0)=eps;
theta0(theta0==pi)=pi-(eps);
theta0(theta0==0)=0+(eps);

% angles of the (n-1) vectors connecting 
% every anchor pair in{n anchors}:
for n=2:numel(r0)

    heading_offsets(n-1)= theta0(n-1)+atan2( r0(n)*sin(theta0(n)-theta0(n-1)),...
                                    r0(n)*cos(theta0(n)-theta0(n-1)) -r0(n-1)  );
    
end

if(rand(1)>0.5)
    kd(1)=0.2;
else
    kd(1)=pi-0.2;
end

% Optimize [anchor distance, anchor heading, initial heading angle:
% r,theta,kd] for every anchor to minimize total path length and sum (angul-
% ar changes at anchor points)

MFE=10000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,'Display','notify',...
    'Algorithm','interior-point');
    
ri=r0;
thetai=theta0;

[optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
    optimize_path_length_and_smoothness(ri,thetai,kd,arena,ka,w1,tol_radius,rg_r,rg_th,opts,dt,r_home);
  
while (exitflag<=0 && (trials_to_optimize<2))
       
        [optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
         optimize_path_length_and_smoothness(ri,thetai,kd,arena,ka,w1,tol_radius,rg_r,rg_th,opts,dt,r_home);

        trials_to_optimize=trials_to_optimize+1;
end


if (~isstruct(output))
    output
end 

optimal_para=output.bestfeasible.x;
[~]=optimizer_obj.my_loss(optimal_para,arena,ka,numel(r0),w1,dt);



if(exitflag==-2 && isempty(output.bestfeasible)) %% the solution is infeasible and 
                                                  % there is no feasible solution from the optimizer
                                                  % trials  
        error('infeasbility')
end


x_op=optimizer_obj.optimized_x;
y_op=optimizer_obj.optimized_y;


% Compute {r,omegas} from the optimized path {x,y} points
 [r,theta]= convert_xy_r_angle(x_op,y_op);
  

end