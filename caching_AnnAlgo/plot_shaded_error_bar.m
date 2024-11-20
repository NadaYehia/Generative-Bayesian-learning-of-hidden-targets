
function plot_shaded_error_bar(data,c)
figure;
trials=size(data,2);
smooth_data=movmean(data(:,1:trials),11,2);
err=std(smooth_data,0,1);
hold on;
shadedErrorBar_std_mean(1:trials, mean(smooth_data) ,err,{'color',c}); hold on;

end