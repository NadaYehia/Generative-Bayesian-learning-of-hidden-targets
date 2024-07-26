
function plot_arena_targets(arena,targets)

    figure(1);
    patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
    hold on,patch([-60 60 60 -60],[190 190 310 310],[0.85 0.9 0.9]);
    hold on,patch([-210 -90 -90 -210],[140 140 260 260],[0.85 0.9 0.9]);
    hold on,patch([90 210 210 90],[140 140 260 260],[0.85 0.9 0.9]);
    hold on,patch([-168 -48 -48 -168],[165 165 285 285],[0.85 0.9 0.9]);
    hold on,patch([48 168 168 48],[165 165 285 285],[0.85 0.9 0.9]);

end