
scale = 2;
trials=size(corr_pop_avg,2);
std_surprise=std(surprise_working_agent(:,1:trials));

f1=figure;
f1.Position=[100 100 200 200];
fontsize(f1,18,"points");
fontname(f1,"Arial");


hold on, shadedErrorBar_std_mean(1:trials, mean(surprise_working_agent(:,1:trials),1) ,std_surprise,{'color',[0.5 0.6 0.5]}); hold on;


scale = 2;
trials=size(corr_pop_avg,2);
std_surprise=std(surprise_flat_agent(:,1:trials));



% hold on, shadedErrorBar_std_mean(1:trials, mean(surprise_flat_agent(:,1:trials),1) ,std_surprise,{'color',[0.6 0.5 0.5]}); hold on;
