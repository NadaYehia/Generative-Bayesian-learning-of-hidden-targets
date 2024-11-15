function new_posterior=posterior_renormalization_set_min_to_eps(eps,posterior)

new_posterior=posterior;
[lind]=find(posterior==0);
[lind_mins]=find( (posterior<=eps) );
C=setdiff(lind_mins,lind);
new_posterior(C)=eps;
new_posterior=(new_posterior)./(sum(new_posterior(:)));


% figure,imagesc(fliplr(Os),As,fliplr(new_posterior'));
% set(gca,'YDir','normal');
% set(gca,'XDir','reverse');
% 
% 
% figure,imagesc(fliplr(Os),As,fliplr(posterior'));
% set(gca,'YDir','normal');
% set(gca,'XDir','reverse');

end
