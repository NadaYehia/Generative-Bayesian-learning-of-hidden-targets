function plot_shaded_error_bar_mean_std(data,c)
scale = 2;
trials=size(data,2);
sem=scale.*(std(data(:,1:trials))./sqrt(size(data,1)));

hold on;
shadedErrorBar_std_mean(1:trials, mean(data(:,1:trials),1) ,sem,{'color',c}); hold on;

end