%plot targets on arena
f=figure(1);    
       f.Position=[100 100 200 200];
       patch(arena.x+400,arena.y,[1 1 1]); hold on;

for target_num=1:5 
    hold on;
    patch(targets(target_num).x+400,targets(target_num).y,[0.85 0.9 0.9]);
   
end

for target_num=1:5
    hold on;
    scatter(targets_xy(target_num,1)+400,targets_xy(target_num,2),30,[0.5 0.5 0.5],'filled');
end