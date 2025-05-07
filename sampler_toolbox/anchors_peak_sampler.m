function [r_anchors,theta_anchors,anchors_no]=anchors_peak_sampler (posterior,Rs,Ths,anchors_no_int,...
    roi_size,pThresh)

% anchors_peak_sampler - Samples anchor points from the peaks of the posterior distribution.
%                        Peaks are identified as local maxima in a 3x3 neighborhood.
%
% Inputs:
%   posterior       - 2D matrix representing the posterior distribution over actions.
%   Rs              - Vector of possible values for the radial dimension (r).
%   Ths             - Vector of possible values for the angular dimension (theta).
%   anchors_no_int  - Initial number of anchor points to sample (used as fallback if the posterior is flat).
%   roi_size        - Size of the local neighborhood for filtering peaks.
%
% Outputs:
%   r_anchors       - Sampled radial coordinates of anchor points.
%   theta_anchors   - Sampled angular coordinates of anchor points.
%   anchors_no      - Number of anchor points after sampling.

    % Shift the posterior matrix in 8 directions to create a 3x3 neighborhood for each pixel.
    % This helps identify local maxima (peaks) by comparing each pixel with its neighbors

%%

% Shift every pixel one position up.
posterior_upshift=circshift(posterior,-1,1);

% Shift every pixel one position down.
posterior_downshift=circshift(posterior,1,1);

% Shift every pixel one position to the right
posterior_rightshift=circshift(posterior,1,2);

% Shift every pixel one position to the left.
posterior_leftshift=circshift(posterior,-1,2);

% Shift every pixel one position up and one position left (northwest).
posterior_nw=circshift(posterior, [-1 -1]);

% Shift every pixel one position up and one position right (northeast).
posterior_ne=circshift(posterior, [-1 1]);

% % Shift every pixel one position down and one position left (southwest).
posterior_sw=circshift(posterior,[1 -1]);

 % Shift every pixel one position down and one position right (southeast).
posterior_se=circshift(posterior, [1  1] );

% Combine all shifted matrices into a 3D tensor (9 layers: original + 8 shifts).
posterior_9Dtensor=cat(3,posterior,...
    posterior_upshift, posterior_downshift,...
    posterior_rightshift, posterior_leftshift,...
    posterior_nw, posterior_ne,...
    posterior_sw,posterior_se);

% Identify true peaks: pixels where the value in the original posterior is greater than
% all corresponding values in the 8 shifted layers. 
nxt_compar_bin_posterior=ones(size(posterior)); % Initialize a binary mask for peaks.

for j=1:8
    % Compare the original posterior layer with each shifted layer.
    posterior_9Dtensor_first_lyr=nxt_compar_bin_posterior.*posterior_9Dtensor(:,:,1);
    posterior_9Dtensor_next_lyr=nxt_compar_bin_posterior.* posterior_9Dtensor(:,:,j+1);
    % Handle NaN values by setting them to 0.
    posterior_9Dtensor_next_lyr(isnan(posterior_9Dtensor_next_lyr))=0;
    posterior_9Dtensor_first_lyr(isnan(posterior_9Dtensor_first_lyr))=0;

    % Update the binary mask to keep only pixels that are greater than the current shifted layer.
    nxt_compar_bin_posterior=(posterior_9Dtensor_first_lyr>posterior_9Dtensor_next_lyr);
    

end
% The final binary mask identifies true peaks
true_peaks=nxt_compar_bin_posterior;

% Find the indices of the true peaks.
[thetas,rs]= find(true_peaks);

% If peaks are found, filter them by proportion of the max posterior value  
 lin_idx=sub2ind(size(posterior),thetas,rs);
 lin_idx_filt=lin_idx( posterior(lin_idx) >= (pThresh*max(posterior(:))) );
 
% If no peaks are found, fall back to sampling N anchors at random from the
% posterior.
if (isempty(lin_idx_filt))
     ancs=anchors_no_int; % Use the initial number of anchors.
     temp_posterior=posterior;
     temp_posterior(isnan(temp_posterior))=0; % Handle NaN values.

     % Sample anchor points proportionally based on the posterior.
     action_array=1:size(posterior,1)*size(posterior,2);
     [l_ind]=datasample(action_array,ancs,'Replace',false,'Weights',temp_posterior(:));

     % Convert linear indices to subscripts (row and column indices).
     [theta_temp_ind,r_temp_ind]=ind2sub(size(posterior),l_ind);

     % Map the sampled indices to their corresponding radial (Rs) and angular (Ths) values.
     r_anchors=Rs(r_temp_ind);
     theta_anchors=Ths(theta_temp_ind);
     anchors_no=size(r_anchors,2); % Number of sampled anchors.

else
     
     % Convert filtered linear indices back to subscripts.
     [thetas_filt,rs_filt]=ind2sub(size(posterior),lin_idx_filt);

     % Sort the filtered peaks by their posterior values in descending order.
     [~,i_sorted_peaks]=sort(posterior(lin_idx_filt),'descend');  
     
     % Select the top half of the peaks (or all if there's only one).
     if (numel(lin_idx_filt)==1)
         an_no=1;
     else
         an_no=floor(numel(lin_idx_filt)/2);
     end

     top_half_peaks=i_sorted_peaks(1:an_no); % Indices of the top half of peaks.

     % Map the selected peaks to their corresponding radial and angular values.

     r_anchors=Rs(rs_filt( top_half_peaks  ));
     theta_anchors=Ths(thetas_filt( top_half_peaks  ));
     anchors_no=size(r_anchors,2); % Number of sampled anchors.


end
  

end