function target_number=set_active_target(k,target_blocks)

non_visited_targets=find(k<=target_blocks);

if(~isempty(non_visited_targets))

    target_number=non_visited_targets(1);
else
    % you visited all targets, this target is the last one.
    target_number= numel(target_blocks);

end



end