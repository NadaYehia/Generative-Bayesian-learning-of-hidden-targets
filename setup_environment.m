function [arena,env,targets]=setup_environment(arena_size,targets_xy,targets_sizes,training_blcks)
% INPUT: arena_size: arena coordinates [x1 x2 y1 y2]
%        targets_xy: targets centres [number of targets (N) x 2]
%        targets_sizes: width and height of targets [number of targets (N)
%        x 2]
%        training_blcks: number of training trials per target.

% OUTPUT: arena: a struct with the x,y coordinates for each of the arena 4
% corners.
%         env: an object of the environment class, its attributes are 
%         arena size, training trials per target, targets centres x&y coordinates
%         targets sizes.
%         targets: N struct, the nth entry has 2 fields {x,y}: the x&y coordinates
%                  of each corner (bottom left, bottom right, top right, top left) 
%                  of the nth target.
%%
% arena struct {x,y}
% x: x-coord of [bottom left corner, bottom right corner, top right corner
%                top left corner]               
arena.x=[arena_size(1) arena_size(2) arena_size(2) arena_size(1)]; 
% y: y-coord of [bottom left corner, bottom right corner, top right corner
%                top left corner]                                                                           
arena.y=[arena_size(3) arena_size(3) arena_size(4) arena_size(4)];

env=environment; % object of environment class
env.blocks=training_blcks; 
env.targets_centers=targets_xy; 
env.targets_dimensions= targets_sizes; 
env.arena_dimensions= arena_size;  
targets=env.setup_targets_coord; 


end