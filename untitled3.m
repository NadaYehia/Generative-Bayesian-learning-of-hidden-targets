figure;
for i=1:20
    hold on, plot(smooth_corr(i,:))
end

%figure,plot([1:15],corr_pop_avg,'r')


figure;
for i=1:15
    imagesc(fliplr(Os),As,fliplr(perm_post(:,:,i,10)'))
     set(gca,'YDir','normal');
     set(gca,'XDir','reverse');
end