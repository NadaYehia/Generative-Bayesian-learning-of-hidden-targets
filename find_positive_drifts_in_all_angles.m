
function positives_drifts=find_positive_drifts_in_all_angles(Drift)

 for angle_idx= 1:size(Drift,1)

     Z = cellfun(@(x)reshape(x,[size(x)]),Drift(angle_idx,:)','un',0);
    angle_speed_drift=cell2mat(Z);

    positives_drifts(angle_idx)= numel( find(angle_speed_drift(:,2)>0) );

 end


end

