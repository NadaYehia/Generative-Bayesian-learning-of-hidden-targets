% tto=tto+1;
%      
%     if(tto>MaxTrials) % if failed to find feasible solution
%         % use input {r,theta anchors} in the next step.  
%         optimal_para=[kd,r0,theta0];
%         [~]=optimizer_obj.my_loss(optimal_para,arena,ka,numel(r0),w2,dt);
% 
%         break;
%     end