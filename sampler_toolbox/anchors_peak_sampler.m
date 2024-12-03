function [mu_anchors,omega_anchors,anchors_no]=anchors_peak_sampler (posterior,anchors_no_int,As,Os,lin_idx_bounds,percentile,dist_criterion)

% find all peaks in a 3x3 neighborhood around every pixel in the posterior

% upward shift: every pixel one position up
posterior_upshift=circshift(posterior,-1,1);

% every pixel one position down
posterior_downshift=circshift(posterior,1,1);

% every pixel one position to the right
posterior_rightshift=circshift(posterior,1,2);

%every pixel one position to the left
posterior_leftshift=circshift(posterior,-1,2);

%every pixel shifted one up and one left
posterior_nw=circshift(posterior, [-1 -1]);

%every pixel shifted one up and one right
posterior_ne=circshift(posterior, [-1 1]);

%every pixel shifted one down and one left
posterior_sw=circshift(posterior,[1 -1]);

%every pixel shifted one down and one right
posterior_se=circshift(posterior, [1  1] );



posterior_9Dtensor=cat(3,posterior,...
    posterior_upshift, posterior_downshift,...
    posterior_rightshift, posterior_leftshift,...
    posterior_nw, posterior_ne,...
    posterior_sw,posterior_se);

% true peaks are the ONLY pixels which values in the first layer are > of all its
% values in the other 8 layers. 
nxt_compar_bin_posterior=ones(size(posterior));

for j=1:8

    posterior_9Dtensor_first_lyr=nxt_compar_bin_posterior.*posterior_9Dtensor(:,:,1);
    posterior_9Dtensor_next_lyr=nxt_compar_bin_posterior.* posterior_9Dtensor(:,:,j+1);
    posterior_9Dtensor_next_lyr(isnan(posterior_9Dtensor_next_lyr))=0;

    nxt_compar_bin_posterior=(posterior_9Dtensor_first_lyr>posterior_9Dtensor_next_lyr);
    

end

true_peaks=nxt_compar_bin_posterior;

[omegas,mus]= find(true_peaks);

lin_idx1=sub2ind(size(posterior),omegas,mus);
not_peaks=find(ismember(lin_idx1,lin_idx_bounds));      
omegas(not_peaks)=[];
mus(not_peaks)=[];

if (isempty(mus))
    l_ind=[];
    ancs=anchors_no_int;
    temp_posterior=posterior;
    temp_posterior(isnan(temp_posterior))=0;
    action_array=[1:size(posterior,1)*size(posterior,2)];
    [l_ind]=datasample(action_array,ancs,'Replace',false,'Weights',temp_posterior(:));
   
     
    [omega_temp_ind,mu_temp_ind]=ind2sub(size(posterior),l_ind);
    
     mu_anchors=As(mu_temp_ind);

    
    omega_anchors=Os(omega_temp_ind);
    anchors_no=size(mu_anchors,2);



else
    
    lin_idx=sub2ind(size(posterior),omegas,mus);

    [vmax]=max(posterior(lin_idx));

    top_anchrs=find(posterior(lin_idx)>=percentile*vmax);

    [sorted_peaks,i_sorted_peaks]=sort(posterior(lin_idx(top_anchrs)),'descend');  
    
    if (numel(top_anchrs)==1)
        an_no=1;
    else
        an_no=floor(numel(top_anchrs)/2);
    end

    peaks_=i_sorted_peaks(1:an_no); % half the peaks ranked

%     rnd_anchrs=randperm(numel(top_anchrs),an_no);

    mu_anchors=As(mus( top_anchrs(peaks_)  ));
    omega_anchors=Os(omegas( top_anchrs(peaks_)  ));
    anchors_no=size(mu_anchors,2);
    
    mu_temp_ind=mus( top_anchrs(peaks_)  );
    omega_temp_ind= omegas( top_anchrs(peaks_)  );
end
  
   %% remove anchors within sigma from each other
%   
%     n=anchors_no;
% 
%     xx= repmat(mu_anchors,n,1);
%     xx=xx(:);
%     xx2=repmat(omega_anchors,n,1);
%     xx2=xx2(:);
% 
%     % yy= (nn)x1 column vector of the first dimension in anchors: mu
%     % repeat mu_anchors' vector 10 times and omega_anchors' vector also
%     % 10x
%     yy=repmat(mu_anchors',n,1);
%     yy2= repmat(omega_anchors',n,1);
% 
%     D1=[xx xx2]-[yy yy2];
%     rng_As=As(end)-As(1);
%     rng_Os=Os(end)-Os(1); 
%     D_= (D1)*[1/((rng_As)^2) 0; 0  1/((rng_Os)^2)]*(D1');
% 
%     D=sqrt(reshape(diag(D_),[n n]));
%     D(D==0)=nan;
%     
%     
%     [vmin,ind_min]= (min(D(:)));
%     % D is nxn matrix. 
%     [r,c]=ind2sub(size(D),ind_min);
% 
%     while ((vmin<dist_criterion))
% 
%         [~,ichs]=max( [posterior(omega_temp_ind(r),mu_temp_ind(r)), posterior(omega_temp_ind(c),mu_temp_ind(c))]);
% 
%         temm=[r,c];
%         ichosen=temm(ichs);
%         
%         
%         %
%         choices=[1,2];
%         not_chosen=choices(choices~=ichs);
%         omega_temp_ind(temm(not_chosen))=[];
%         mu_temp_ind(temm(not_chosen))=[];
% 
%         mu_anchors(temm(not_chosen))=[];
%         omega_anchors(temm(not_chosen))=[];
%      
%         %
%        
%         n=size(mu_anchors,2);
% 
%         % recalc D 
%         xx= repmat(mu_anchors,n,1);
%         xx=xx(:);
%         xx2=repmat(omega_anchors,n,1);
%         xx2=xx2(:);
%     
%         yy=repmat(mu_anchors',n,1);
%         yy2= repmat(omega_anchors',n,1);
%     
%         D1=[xx xx2]-[yy yy2];
%     
%         D_= (D1)*[1/((rng_As)^2) 0; 0  1/((rng_Os)^2)]*(D1');
%     
%         D=sqrt(reshape(diag(D_),[n n]));
%         D(D==0)=nan;
%         
%         
%         [vmin,ind_min]= (min(D(:)));
%          % D is nxn matrix. 
%         [r,c]=ind2sub(size(D),ind_min);
% 
%     end
%    anchors_no=size(mu_anchors,2);


end