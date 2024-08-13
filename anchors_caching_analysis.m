agent=2;
r_ancs=r_anchors_pop(agent);

theta_ancs=om_anchors_pop(agent);
figure;

for t=1:200
    rs=r_ancs{1}{t};
    thetas=theta_ancs{1}{t};
    hold on,scatter(thetas(2:end-1),rs(2:end-1),30,'filled','MarkerFaceAlpha',0.5);
   set(gca,'XDir','reverse');
end