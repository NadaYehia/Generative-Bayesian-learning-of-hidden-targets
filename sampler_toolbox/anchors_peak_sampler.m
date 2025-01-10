function [r_anchors,theta_anchors,anchors_no]=anchors_peak_sampler (posterior,anchors_no_int,Rs,Ths,percentile)

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

[thetas,rs]= find(true_peaks);


if (isempty(rs))
     l_ind=[];
     ancs=anchors_no_int;
     temp_posterior=posterior;
     temp_posterior(isnan(temp_posterior))=0;
     action_array=[1:size(posterior,1)*size(posterior,2)];
     [l_ind]=datasample(action_array,ancs,'Replace',false,'Weights',temp_posterior(:));
     [theta_temp_ind,r_temp_ind]=ind2sub(size(posterior),l_ind);
     r_anchors=Rs(r_temp_ind);
     theta_anchors=Ths(theta_temp_ind);
     anchors_no=size(r_anchors,2);

else
     lin_idx=sub2ind(size(posterior),thetas,rs);
     [vmax]=max(posterior(lin_idx));
     top_anchrs=find(posterior(lin_idx)>=percentile*vmax);
     [sorted_peaks,i_sorted_peaks]=sort(posterior(lin_idx(top_anchrs)),'descend');  
    
     if (numel(top_anchrs)==1)
         an_no=1;
     else
         an_no=floor(numel(top_anchrs)/2);
     end

     peaks_=i_sorted_peaks(1:an_no); % half the peaks ranked
     r_anchors=Rs(rs( top_anchrs(peaks_)  ));
     theta_anchors=Ths(thetas( top_anchrs(peaks_)  ));
     anchors_no=size(r_anchors,2);
end
  

end