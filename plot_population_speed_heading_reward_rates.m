scale = 1;
trials=size(corr_pop_avg,2);

figure;
subplot(1,4,3), shadedErrorBar(1:trials, mean(corr_pop_avg(:,1:trials)) ,std( corr_pop_avg(:,1:trials))./sqrt(size(corr_pop_avg,1) )./scale,{'color','g'}); hold on;

hold on, subplot(1,4,1),shadedErrorBar(1:trials, mean(mu_pop_avg(:,1:trials)) ,std( mu_pop_avg(:,1:trials))./sqrt(size(mu_pop_avg,1) )./scale,{'color','g'}); hold on;

hold on, subplot(1,4,4), shadedErrorBar(1:trials, mean(anchors_no_pop(:,1:trials)) ,std( anchors_no_pop(:,1:trials))./sqrt(size(anchors_no_pop,1) )./scale,{'color','g'}); hold on;

hold on, subplot(1,4,2), shadedErrorBar(1:trials, mean(hd_pop_avg(:,1:trials)) ,std( hd_pop_avg(:,1:trials))./sqrt(size(hd_pop_avg,1) )./scale,{'color','g'}); hold on
    