function [bw,is,js]=convert_poly_to_mask(r_boundary,theta_boundary,sizee,Rs,Ths,max_x_wall)

dR=Rs(2)-Rs(1);
for s=1:numel(r_boundary)
    [~,is(s)]=(min(abs(Ths-theta_boundary(s)))); % finding the bin numbers of the binned theta boundary
    js(s)=floor(r_boundary(s)/dR)+1; % finding the bin numbers for the radial boundaries
    
end

% initalize binary mask as the size of the prior
bw=zeros(sizee);
% set pixels inside the indices of the arena boundaries polygon to 1 and 0s
% outside
[x, y] = meshgrid(1:sizee(1), 1:sizee(2));
inside = inpolygon(x, y, js, is);
bw(inside)=1;
% set the locations along any angle from R(1) up to the bottom right and bottom left corners radii to 1. 
bw(:,1:max_x_wall)=1;

end
