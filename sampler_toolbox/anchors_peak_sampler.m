function [mu_anchors,omega_anchors,anchors_no]=anchors_peak_sampler (posterior,anchors_no_int,As,Os,dist_criterion,r_bounds,c_bounds)

percentile=0.6;
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


    nxt_compar_bin_posterior=(posterior_9Dtensor_first_lyr>posterior_9Dtensor_next_lyr);

    
end

true_peaks=nxt_compar_bin_posterior;

[omegas,mus]= find(true_peaks);

lin_idx1=sub2ind(size(posterior),omegas,mus);

lin_idx_bounds=sub2ind(size(posterior),r_bounds,c_bounds);
% add the indices for the first row, last row,, first column
ind_first_row= sub2ind(size(posterior),repmat(1,size(posterior,2),1),(1:size(posterior,2))');
ind_last_row= sub2ind(size(posterior),repmat(501,size(posterior,2),1),(1:size(posterior,2))');


lin_idx_bounds=[lin_idx_bounds, [1:size(posterior,1)], ind_first_row', ind_last_row'];
not_peaks=find(ismember(lin_idx1,lin_idx_bounds));      
omegas(not_peaks)=[];
mus(not_peaks)=[];

if (isempty(mus))
    l_ind=[];
    ancs=anchors_no_int;
    while(numel(l_ind)~=anchors_no_int)
        
        action_array=[1:size(posterior,1)*size(posterior,2)];
        [temp]=datasample(action_array,ancs,'Replace',false,'Weights',(posterior(:)));
   
        not_peaks=find(ismember(temp,lin_idx_bounds));
        temp(not_peaks)=[];
        l_ind=[l_ind,temp];
        ancs=anchors_no_int-numel(l_ind);

    end
     
    [omega_temp_ind,mu_temp_ind]=ind2sub(size(posterior),l_ind);
    
     mu_anchors=As(mu_temp_ind);

    
    omega_anchors=Os(omega_temp_ind);
    anchors_no=size(mu_anchors,2);



else
    
    lin_idx=sub2ind(size(posterior),omegas,mus);

    [vmax]=max(posterior(lin_idx));

    top_anchrs=find(posterior(lin_idx)>=percentile*vmax);

    an_no=round(numel(top_anchrs)/2);


    rnd_anchrs=randperm(numel(top_anchrs),an_no);

    mu_anchors=As(mus( top_anchrs(rnd_anchrs)  ));
    omega_anchors=Os(omegas( top_anchrs(rnd_anchrs)  ));
    anchors_no=size(mu_anchors,2);

end

end