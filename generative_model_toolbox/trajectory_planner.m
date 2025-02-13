function [r,theta,x_op,y_op,exitflag]=trajectory_planner(r_anchors,theta_anchors,env,Os,Rs,...
ka,w1,w2,w3,tol_radius,r_home)


%% Initalizations:

dt=0.01; % time discretization for simulating a path between 2 anchors.
arena=env.arena_dimensions;
rg_r=abs(Rs(end)-Rs(1));
rg_th=abs(Os(end)-Os(1));
eps=1e-4;
%kd=zeros(1,numel(r_anchors)-1); % (n-1) row vector of the initial 
                                            % heading angles of the (n-1) 
                                            % segments connecting between 
                                            % every anchor pair in {n}anchors.



theta0=theta_anchors+(pi/2);
r0=r_anchors;
r0(r0==0)=eps;
theta0(theta0==pi)=pi-(eps);
theta0(theta0==0)=0+(eps);

% angles of the (n-1) vectors connecting every anchor pair in{n anchors}:
for n=2:numel(r0)

    heading_offsets(n-1)= theta0(n-1)+atan2( r0(n)*sin(theta0(n)-theta0(n-1)),...
                                    r0(n)*cos(theta0(n)-theta0(n-1)) -r0(n-1)  );
    
end

if(rand(1)>0.5)
    kd(1)=heading_offsets(1)-eps;
else
    kd(1)=(-(pi-heading_offsets(1)))+eps;
end

% kd(1)=randn(1)*2;
%% Optimize [anchor distance, anchor heading, initial heading angle:
% r,theta,kd] for every anchor to minimize total path length and sum (angul-
% ar changes at anchor points)

MFE=10000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,'Display','notify',...
    'Algorithm','interior-point');
    
ri=r0;
thetai=theta0;

[optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
    optimize_path_length_and_smoothness(ri,thetai,kd,arena,ka,w1,w2,w3,tol_radius,rg_r,rg_th,opts,dt,r_home);
  
if (exitflag<=0)
        
        optimal_para=[kd,r0,theta0];
        [~]=optimizer_obj.my_loss(optimal_para,arena,ka,numel(r0),w1,dt);

else

    if(~isempty(output.bestfeasible))
        optimal_para=output.bestfeasible.x;
        [loss1]=optimizer_obj.my_loss(optimal_para,arena,ka,numel(r0),w1,dt);
    end
end

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