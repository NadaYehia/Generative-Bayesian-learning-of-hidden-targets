
function plot_shaded_error_bar(data,c)
figure;
trials=size(data,2);
smooth_data=movmean(data(:,1:trials),11,2);
c1=[0.12156862745098039 0.4666666666666667 0.7058823529411765];
c2=[1 0.4980392156862745 0.054901960784313725];
c3=[0.17254901960784313 0.6274509803921569 0.17254901960784313];
c4=[0.5019607843137255 0 0.5019607843137255];
c5=[0.6627450980392157 0.6627450980392157 0.6627450980392157];
colors=[c1;c2;c3;c4;c5];
for i=1:5
    err=std(smooth_data(:,1+((i-1)*100):i*100),0,1);
    hold on;
    shadedErrorBar_std_mean(1+((i-1)*100):i*100, mean(smooth_data(:,1+((i-1)*100):i*100)) ,err,{'color',colors(i,:)}); hold on;
end
end