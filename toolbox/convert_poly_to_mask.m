function [bw,is,js]=convert_poly_to_mask(rs,omega,sizee,Rs,Omegas)



for s=1:numel(rs)
    [~,is(s)]=(min(abs(Omegas-omega(s))));
    [~,js(s)]=(min(abs(Rs-rs(s))));
    
end

bw_boundary=poly2mask(js,is,sizee(1),sizee(2));
% to eliminate the issues with poly2mask fun
bw_flipped=flipud(bw_boundary); % 
bw= bw_boundary | bw_flipped;


end
