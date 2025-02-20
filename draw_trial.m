function draw_trial(Ths,Rs,prior,theta_anchors,r_anchors,arena,targets,target_num,pos_x,pos_y)
       
       figure(2);
       h2=imagesc(Ths,Rs,(prior'));
       set(h2, 'AlphaData', ~isnan(h2.CData)) % visualize nans entires in the posterior as transparent.
       set(gca,'YDir','normal');
       set(gca,'XDir','reverse');
       xticks([-pi/2 -pi/3 -pi/4 -pi/8 0 pi/8 pi/4 pi/3 pi/2])
       xticklabels({'-\pi/2','-\pi/3','-\pi/4','-\pi/8','0','\pi/8','\pi/4','\pi/3','\pi/2'})
       set(gca,'TickDir','out')
       hold on, scatter(theta_anchors,r_anchors,30,'r','filled');
       hold off;
       colorbar;
       f=figure(3);    
       f.Position=[100 100 200 200];
       patch(arena.x,arena.y,[1 1 1]); hold on;
       patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);
       hold on, plot(pos_x',pos_y','k','LineWidth',1.5);
       hold on,scatter(pos_x,pos_y,5,1:numel(pos_y),'filled');
       set(gca,'XTick',[]);
       set(gca,'YTick',[]);
       
end