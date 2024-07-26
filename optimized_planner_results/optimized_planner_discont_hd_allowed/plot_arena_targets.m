
function plot_arena_targets(arena,targets)

    figure(1);
    patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
    
    for tr=1:size(targets,2)

        hold on,patch(targets(tr).x,targets(tr).y,[0.85 0.9 0.9]);
    end
end