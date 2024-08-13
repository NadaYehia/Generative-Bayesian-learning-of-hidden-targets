
scale = 2;
trials=size(corr_pop_avg,2);
smooth_corr_pop_avg=movmean(corr_pop_avg,5,2);

sem_acc= scale.*(std(smooth_corr_pop_avg(:,1:trials))./sqrt(size(smooth_corr_pop_avg,1)));

sem_r=  scale.*(std(r_pop_avg(:,1:trials))./sqrt(size(r_pop_avg,1)));

sem_anchorsno= scale.*(std(anchors_no_pop(:,1:trials))./sqrt(size(anchors_no_pop,1)));

sem_hd= scale.*(std(hd_pop_avg(:,1:trials))./sqrt(size(hd_pop_avg,1)));


figure;
subplot(1,4,3), shadedErrorBar_std_mean(1:trials, mean(smooth_corr_pop_avg(:,1:trials),1) ,sem_acc ,{'color','g'}); hold on;
ylim([0 1]);
ylabel('reward rate');

hold on, subplot(1,4,1),shadedErrorBar_std_mean(1:trials, mean(r_pop_avg(:,1:trials),1) ,sem_r,{'color','g'}); hold on;
ylim([1 1000])
ylabel('radial distance from home port');

hold on, subplot(1,4,4), shadedErrorBar_std_mean(1:trials, mean(anchors_no_pop(:,1:trials),1) ,sem_anchorsno,{'color','g'}); hold on;
ylim([1 10])
ylabel('number of anchors (pauses)');

hold on, subplot(1,4,2), shadedErrorBar_std_mean(1:trials, mean(hd_pop_avg(:,1:trials),1) ,sem_hd,{'color','g'}); hold on
ylim([-pi/2 pi/2])   
ylabel('heading angle from home port')