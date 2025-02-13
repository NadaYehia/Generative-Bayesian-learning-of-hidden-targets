
clearvars anchors_no r_anchors theta_anchors Gsol exitflg anchors rs_new thetas_new posx_new posy_new
tol_radius=0.01;
w2_L=1e2;
for i=1:100

    initial_ancs=randperm(1,1);
    [r_anchors,theta_anchors,anchors_no]= Sampling_next_actions(prior_global,sampler,initial_ancs,Rs,Ths,merging_criterion,r_bounds,theta_bounds,...
        Rs(min(c_home)),0,...
        0,0,0);

        if(anchors_no>1)
            [Gsol,~,~]=connect_anchors_tsp([ 0 r_anchors]',[ 0 theta_anchors]',anchors_no+1,Rs,Ths);       
            [r_anchors,theta_anchors]=reorder_actions_anchors([0 r_anchors],[ 0 theta_anchors],Gsol);
            [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg(i)]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_radius,Rs(min(c_home)));
            
        else
            r_anchors=[0 r_anchors 0];
            theta_anchors=[0, theta_anchors, 0];
            [rs_new,thetas_new,pos_xnew,pos_ynew,exitflg(i)]= trajectory_planner(r_anchors,theta_anchors,env,clearnce,Ths,Rs,ka,w2_L,tol_radius,Rs(min(c_home)));
        end
  
     anchors{i}.r=r_anchors;
     anchors{i}.theta=theta_anchors;

end