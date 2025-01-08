function [bw,is,js]=convert_poly_to_mask(r_boundary,theta_boundary,sizee,Rs,Ths)



for s=1:numel(r_boundary)
    [~,is(s)]=(min(abs(Ths-theta_boundary(s))));
    [~,js(s)]=(min(abs(Rs-r_boundary(s))));
    
end

bw_boundary=poly2mask(js,is,sizee(1),sizee(2));
% to eliminate the issues with poly2mask fun
bw_flipped=flipud(bw_boundary); % 
bw= bw_boundary | bw_flipped;

end
