function [flg,Sw,Sf]=surprise_Ann(outcome, current_posterior,current_actions,Rs,Os,sigma_ridge,h0,...
    env,min_radius_around_home,wrkrs,arena_home_mask)
    
mus_curr=current_actions(:,2);
omegas_curr=current_actions(:,1);

% likelihood of current traj
L1=place_field_map_for_likelihood(mus_curr,omegas_curr,sigma_ridge,Rs,Os,wrkrs);
naned_arena_home_mask=1-arena_home_mask;
naned_arena_home_mask(find(naned_arena_home_mask))=nan;
arena_home_mask=arena_home_mask+naned_arena_home_mask;

L1=L1.*arena_home_mask;
L1= (L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) );

if(outcome==1)
    L1=L1;
else
    L1=(1-L1);
end
%%

flat_prior=arena_home_mask./(nansum(arena_home_mask(:)));

p_flat_2d= (flat_prior.*L1);
p_flat=nansum(p_flat_2d(:));
%%
p_working_2d=current_posterior.*L1;
p_working=nansum(p_working_2d(:));


Sw=-log2(p_working);
Sf=-log2(p_flat);
flg=(Sw/Sf)>h0;

end