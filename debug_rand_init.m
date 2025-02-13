

for s=1:10000

dt=0.01; % time discretization for simulating a path between 2 anchors.
arena=env.arena_dimensions;
rg_r=abs(Rs(end)-Rs(1));
rg_th=abs(Ths(end)-Ths(1));
eps=1e-3;
opt_tr=0;
kd=rand(1,numel(r_anchors)-1); % (n-1) row vector of the initial 
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

%% Optimize [anchor distance, anchor heading, initial heading angle:
% r,theta,kd] for every anchor to minimize total path length and sum (angul-
% ar changes at anchor points),'PlotFcn','optimplotfval'

MFE=300000;
Itrs=1000;
opts=optimoptions("fmincon","MaxFunctionEvaluations",MFE,"MaxIterations",Itrs,'Display','notify',...
    'Algorithm','interior-point');
    
  
w2=w2_L;
ri=r0;
thetai=theta0;
   
[optimal_para,fval,exitflag,output,~,~,~,optimizer_obj]=...
    optimize_path_length_and_smoothness(ri,thetai,kd,arena,ka,w2,tol_radius,rg_r,rg_th,opts,dt,Rs(min(c_home)));
end