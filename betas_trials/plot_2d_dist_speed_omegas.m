%% build 2D distribution for omegas and speeds for any noise model at convergence

function P= plot_2d_dist_speed_omegas(As,Os,speed,omegas)
%nx 50 trials/runs x 2 (speed and angle)
joint_speed_omegas= cat(3,speed,omegas);

P=zeros(numel(Os),numel(As));

for agent=1:size(joint_speed_omegas,1)

    temp=joint_speed_omegas(agent,:,:); % 1x50x2
    
    for sample=1:size(joint_speed_omegas,2)

        temp_speed= temp(1,sample,1);
        temp_angle= temp(1,sample,2);
        
        [~,idx_o]=min( abs(Os-temp_angle) );

        [~,idx_a]=min( abs(As-temp_speed) );

        P(idx_o,idx_a)=P(idx_o,idx_a)+1;
    end

end



end

