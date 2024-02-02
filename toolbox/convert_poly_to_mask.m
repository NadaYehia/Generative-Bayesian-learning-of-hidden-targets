function [bw_boundary,is,js]=convert_poly_to_mask(mu,omega,sizee,As,Omegas)



for s=1:numel(mu)
    [~,is(s)]=(min(abs(Omegas-omega(s))));
    [~,js(s)]=(min(abs(As-mu(s))));
    
end

bw_boundary=poly2mask(js,is,sizee(1),sizee(2));
bw_boundary(:,1)=1;
end