function L1=Dist_from_ridge_prob_conv(x_,y_,sigma,As,Omegas)


% eliminate speeds outside our posterior action space.
[bx1]=find(x_<As(1));
[bx2]=find(x_>As(end));

bx=[bx1, bx2];

x_(bx)=[];
y_(bx)=[];

% eliminate angles outside our posterior actions space.
[by1]=find(y_<Omegas(1));
[by2]=find(y_>Omegas(end));

by=[by1,by2];

x_(by)=[];
y_(by)=[];

%
L=zeros(size(Omegas,2),size(As,2));

for s=1:numel(y_)
    [~,is]=(min(abs(Omegas-y_(s))));
    [~,js]=(min(abs(As-x_(s))));
    L(is,js)=1;

end


sz=sigma*50;
h=fspecial('gaussian',sz,sigma);
L1=conv2(L,h,'same');







end