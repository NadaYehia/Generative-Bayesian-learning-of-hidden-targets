function [bw,is,js]=convert_poly_to_mask(r_boundary,theta_boundary,sizee,Rs,Ths,max_x_wall)

dR=Rs(2)-Rs(1);
for s=1:numel(r_boundary)
    [~,is(s)]=(min(abs(Ths-theta_boundary(s))));
    js(s)=floor(r_boundary(s)/dR)+1; 
    
end

bw=zeros(sizee);
[x, y] = meshgrid(1:sizee(1), 1:sizee(2));
inside = inpolygon(x, y, js, is);
bw(inside)=1;
bw(:,1:max_x_wall)=1;

end
