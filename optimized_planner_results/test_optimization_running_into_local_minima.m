figure;
for i=1:500
r_anchors=[0 321 0]
omega_anchors=[2.53 2.42 2.53]
omega_anchors=omega_anchors-(pi/2)
[r,omega,x_op,y_op]=connect_actions_r_theta_with_optimized_planner(r_anchors,omega_anchors,env,clearnce,Drift,Os,Rs,0,0,...
0,drift_fac,ka,50,1e-7);
hold on, plot(x_op,y_op);
end