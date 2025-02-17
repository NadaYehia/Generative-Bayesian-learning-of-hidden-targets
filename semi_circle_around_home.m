function [prior_exc_home,c]=semi_circle_around_home(prior,min_radius,Rs)

[~,c]=min(abs(Rs-min_radius));

prior_exc_home=ones(size(prior));
prior_exc_home(:,1:c)=0;


end