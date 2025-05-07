
scale = 2;
trials=size(corr_pop_avg,2);
smooth_corr_pop_avg=movmean(corr_pop_avg,11,2);


% sem_acc= scale.*(std(smooth_corr_pop_avg(:,1:trials))./sqrt(size(smooth_corr_pop_avg,1)));



% sem_r=  scale.*(std(r_pop_avg(:,1:trials))./sqrt(size(r_pop_avg,1)));

% sem_anchorsno= scale.*(std(anchors_no_pop(:,1:trials))./sqrt(size(anchors_no_pop,1)));

% sem_hd= scale.*(std(hd_pop_avg(:,1:trials))./sqrt(size(hd_pop_avg,1)));


f1=figure;
f1.Position=[100 100 200 200];
fontsize(f1,18,"points");
fontname(f1,"Arial");
c1=[0.12156862745098039 0.4666666666666667 0.7058823529411765];
c2=[1 0.4980392156862745 0.054901960784313725];
c3=[0.17254901960784313 0.6274509803921569 0.17254901960784313];
c4=[0.5019607843137255 0 0.5019607843137255];
c5=[0.6627450980392157 0.6627450980392157 0.6627450980392157];
colors=[c1;c2;c3;c4;c5];
for i=1:5
    std_acc=std(smooth_corr_pop_avg(:,1+((i-1)*100):i*100));    
    hold on, shadedErrorBar_std_mean(1+((i-1)*100):i*100, mean(smooth_corr_pop_avg(:,1+((i-1)*100):i*100),1) ,std_acc ,{'color',colors(i,:)}); hold on;
end

% plot_shaded_error_bar(smooth_corr_pop_avg,[0.5 0.6 0.5])
ylim([0 1]);
ylabel('reward rate');
