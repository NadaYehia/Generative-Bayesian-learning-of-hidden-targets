function [flg,Sw,Sf]=surprise(L1,outcome, current_posterior,h0,arena_home_mask)
    

if(outcome==0)
 
    L1=(1-L1);
end


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