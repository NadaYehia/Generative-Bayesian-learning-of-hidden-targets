%plot distribution of final run speeds and heading angle in single anchor
%solution. 





%% plot the average trace for single anchors speeds and heading angle at all
% trials

%find agents with #anchors converged to 1 after tstart trials in
t_start=50;
slice_anchors_t_start_end=anchors_no_pop(:,t_start:end);
single_ancrs_ids=find(all(slice_anchors_t_start_end==1,2));

scale = 1;
trials=size(corr_pop_avg,2);
%% 

figure;
subplot(1,4,3), shadedErrorBar(1:trials, mean(corr_pop_avg(single_ancrs_ids,1:trials)) ,std( corr_pop_avg(single_ancrs_ids,1:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','g'}); hold on;
ylim([0 1]);
hold on, subplot(1,4,1),shadedErrorBar(1:trials, mean(mu_pop_avg(single_ancrs_ids,1:trials)) ,std( mu_pop_avg(single_ancrs_ids,1:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','g'}); hold on;
ylim([1 1000])
hold on, subplot(1,4,4), shadedErrorBar(1:trials, mean(anchors_no_pop(single_ancrs_ids,1:trials)) ,std( anchors_no_pop(single_ancrs_ids,1:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','g'}); hold on;
ylim([1 10])
hold on, subplot(1,4,2), shadedErrorBar(1:trials, mean(hd_pop_avg(single_ancrs_ids,1:trials)) ,std( hd_pop_avg(single_ancrs_ids,1:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','g'}); hold on
ylim([-pi/2 pi/2])    

%% plot the anchors in red on top of the inferred angle from trajctory
mu_anchors_single_ancs=mu_anchors_pop(single_ancrs_ids);
hd_anchors_single_ancs=om_anchors_pop(single_ancrs_ids);

%now loop in each cell array and plot the single anchors heading from t_50
%to the end in angle from traj and angle from anchors 
for s=1:size(hd_anchors_single_ancs,2)
    temp=hd_anchors_single_ancs{s};

    for run=t_start:trials
        anchors_hds=temp{run};
        hd_ancs_pop(s,run)=anchors_hds(2);

    end
end



figure;
hold on, shadedErrorBar(t_start:trials, mean(hd_pop_avg(single_ancrs_ids,t_start:trials)) ,std( hd_pop_avg(single_ancrs_ids,t_start:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','g'}); hold on
figure; hold on, shadedErrorBar(t_start:trials, mean(hd_ancs_pop(:,t_start:trials)) ,std( hd_ancs_pop(:,t_start:trials))./sqrt(numel(single_ancrs_ids) )./scale,{'color','r'}); hold on

