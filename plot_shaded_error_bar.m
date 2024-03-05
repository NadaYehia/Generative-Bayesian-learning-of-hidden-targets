
function plot_shaded_error_bar(data)
scale = 1;
trials=size(data,2);

figure;
shadedErrorBar(1:trials, mean(data(:,1:trials)) ,std( data(:,1:trials))./sqrt(size(data,1) )./scale,{'color','b'}); hold on;
end