function [prior_exc_home,is,js]=semi_circle_around_home(prior,min_radius,Rs,Omegas)

domega=abs(Omegas(1)-Omegas(2));
dradius= abs(Rs(1)-Rs(2));

radiuss=0:dradius:min_radius;

x=-pi/2:domega:pi/2;


y=repmat(min_radius,1,numel(x));


for s=1:numel(x)
    [~,is(s)]=(min(abs(Omegas-x(s))));
    [~,js(s)]=(min(abs(Rs-y(s))));
end
home_highlighted=zeros(size(prior));
home_highlighted(:,1:js(1))=1;
prior_exc_home=~(home_highlighted);


end