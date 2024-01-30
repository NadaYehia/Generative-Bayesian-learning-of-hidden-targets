
function [intercept,sigma_ridge,arena,blocks,target,target2 ]=env_settings(interceptMode,uncertainityInActions,TrialsT1T2,xc1,yc1,xc2,yc2,w1,h1,...
    w2,h2)

intercept= interceptMode;

sigma_ridge=uncertainityInActions;

  
arena.dims = [-400 400 0 600];
blocks = TrialsT1T2;

arena.x = arena.dims([1 2 2 1]);
arena.y = arena.dims([3 3 4 4]);

% target A specs
target.cntr = [xc1 yc1];
target.width = [w1 h1];
target.x = [target.cntr(1)-target.width(1) target.cntr(1)+target.width(1) target.cntr(1)+target.width(1) target.cntr(1)-target.width(1)];
target.y = [target.cntr(2)-target.width(2) target.cntr(2)-target.width(2) target.cntr(2)+target.width(2) target.cntr(2)+target.width(2)];

% target B specs
target2.cntr = [xc2 yc2];
target2.width = [w2 h2];
target2.x = [target2.cntr(1)-target2.width(1) target2.cntr(1)+target2.width(1) target2.cntr(1)+target2.width(1) target2.cntr(1)-target2.width(1)];
target2.y = [target2.cntr(2)-target2.width(2) target2.cntr(2)-target2.width(2) target2.cntr(2)+target2.width(2) target2.cntr(2)+target2.width(2)];

figure(1);
patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
patch(target.x,target.y,[0.85 0.9 0.9]);
patch(target2.x,target2.y,[0.9 0.9 0.85]);
figure(2);

end

