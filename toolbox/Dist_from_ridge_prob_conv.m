function L1=Dist_from_ridge_prob_conv(x_,y_,sigma,As,Omegas)

L=zeros(size(Omegas,2),size(As,2));

for s=1:numel(y_)
    [~,is]=(min(abs(Omegas-y_(s))));
    [~,js]=(min(abs(As-x_(s))));
    L(is,js)=1;

end


sz=1*max(size(L));
h=fspecial('gaussian',sz,sigma);
L1=conv2(L,h,'same');







end