scale = 1;
trials=size(corr_pop_avg,2);
smooth_corr_pop_avg=movmean(corr_pop_avg,5,2);

prc_accuracy=prctile(smooth_corr_pop_avg,[90 10],1);
prc_r=prctile(r_pop_avg,[90 10],1);
prc_anchors_no=prctile(anchors_no_pop(:,1:trials),[90 10],1);
prc_hd=prctile(hd_pop_avg,[90 10],1);


figure;
subplot(1,4,3), shadedErrorBar(1:trials, median(smooth_corr_pop_avg(:,1:trials),1) ,prc_accuracy ,{'color','g'}); hold on;
ylim([0 1]);
ylabel('reward rate');

hold on, subplot(1,4,1),shadedErrorBar(1:trials, median(r_pop_avg(:,1:trials),1) ,prc_r,{'color','g'}); hold on;
ylim([1 1000])
ylabel('radial distance from home port');

hold on, subplot(1,4,4), shadedErrorBar(1:trials, median(anchors_no_pop(:,1:trials),1) ,prc_anchors_no,{'color','g'}); hold on;
ylim([1 10])
ylabel('number of anchors (pauses)');

hold on, subplot(1,4,2), shadedErrorBar(1:trials, median(hd_pop_avg(:,1:trials),1) ,prc_hd,{'color','g'}); hold on
ylim([-pi/2 pi/2])   
ylabel('heading angle from home port')