
function plot_shaded_error_bar(data,c)
scale = 1;
trials=size(data,2);
prc_err=prctile(data,[90 10],1);

hold on;
shadedErrorBar(1:trials, median(data(:,1:trials),1) ,prc_err,{'color',c}); hold on;
end