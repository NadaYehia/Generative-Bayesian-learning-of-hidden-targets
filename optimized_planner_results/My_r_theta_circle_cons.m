function [c,ceq]=My_r_theta_circle_cons (p,no_anchors,tol_radius,initial_anchors_r,initial_anchors_theta,rg_r,rg_th)
    
ceq=[];
r_norm=(initial_anchors_r-p(no_anchors:(2*no_anchors)-1))./rg_r;
th_norm=(initial_anchors_theta-p(2*no_anchors:end))./rg_th;
c= ( r_norm .^2)+...
    (th_norm.^2)-(tol_radius^2);



