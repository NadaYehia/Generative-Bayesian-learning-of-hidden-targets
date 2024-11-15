
scale = 2;
trials=size(corr_pop_avg,2);
Sw_sf=(surprise_working_agent(:,1:trials)./surprise_flat_agent(:,1:trials));

std_surprise=std(Sw_sf(:,1:trials));

f1=figure;
f1.Position=[100 100 200 200];
fontsize(f1,18,"points");
fontname(f1,"Arial");


hold on, shadedErrorBar_std_mean(1:trials, mean(Sw_sf,1) ,std_surprise,{'color','b'}); hold on;
