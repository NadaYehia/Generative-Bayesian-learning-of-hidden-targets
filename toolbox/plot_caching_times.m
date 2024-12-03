
scale = 2;
trials=size(corr_pop_avg,2);
std_caching=std(caching_times(:,1:trials));

f1=figure;
f1.Position=[100 100 200 200];
fontsize(f1,18,"points");
fontname(f1,"Arial");


hold on, shadedErrorBar_std_mean(1:trials, mean(caching_times(:,1:trials),1) ,std_caching,{'color',[0.5 0.6 0.5]}); hold on;
ylim([0 1])
ylabel('fraction of agents cached at times t');
