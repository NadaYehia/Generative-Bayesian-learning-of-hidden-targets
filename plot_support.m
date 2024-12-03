scale = 2;
trials=size(corr_pop_avg,2);
% (:,101:200)

% sem_acc= scale.*(std(smooth_corr_pop_avg(:,1:trials))./sqrt(size(smooth_corr_pop_avg,1)));

% sem_r=  scale.*(std(r_pop_avg(:,1:trials))./sqrt(size(r_pop_avg,1)));

% sem_anchorsno= scale.*(std(anchors_no_pop(:,1:trials))./sqrt(size(anchors_no_pop,1)));

% sem_hd= scale.*(std(hd_pop_avg(:,1:trials))./sqrt(size(hd_pop_avg,1)));
log_support=log10(posteriors_support);
% sem_log_support= scale.*(std(log_support(:,1:trials))./sqrt(size(log_support,1)));
std_log_support=std(log_support(:,1:trials));

f1=figure;
f1.Position=[100 100 200 200];
fontsize(f1,18,"points");
fontname(f1,"Arial");
% hold on, subplot(1,2,1), shadedErrorBar_std_mean(1:trials, mean(smooth_corr_pop_avg(:,1:trials),1) ,sem_acc ,{'color','g'}); hold on;
% ylim([0 1]);
% ylabel('reward rate');

% hold on, subplot(1,4,1),shadedErrorBar_std_mean(1:trials, mean(r_pop_avg(:,1:trials),1) ,sem_r,{'color','g'}); hold on;
% ylim([1 1000])
% ylabel('radial distance from home port');

hold on, shadedErrorBar_std_mean(1:trials, mean(log_support(:,1:trials),1) ,std_log_support,{'color',[0.5 0.6 0.5]}); hold on;
ylim([0 6])
ylabel('posterior support');

% hold on, subplot(1,4,2), shadedErrorBar_std_mean(1:trials, mean(hd_pop_avg(:,1:trials),1) ,sem_hd,{'color','g'}); hold on
% ylim([-pi/2 pi/2])   
% ylabel('heading angle from home port')

