function [prior_exc_home,is,js]=semi_circle_around_home(prior,min_radius,Rs,Ths)

domega=abs(Ths(1)-Ths(2));

x=-pi/2:domega:pi/2;


y=repmat(min_radius,1,numel(x));


for s=1:numel(x)
    [~,is(s)]=(min(abs(Ths-x(s))));
    [~,js(s)]=(min(abs(Rs-y(s))));
end
home_highlighted=zeros(size(prior));
home_highlighted(:,1:js(1))=1;
prior_exc_home=~(home_highlighted);


end