function [r_anchors,theta_anchors,n]=anchors_prop_sampler_wnn_merge (posterior,n,Rs,Ths,dist_criterion)

% anchors_prop_sampler_wnn_merge - Samples and merges anchor points based on a posterior distribution.
%
% Inputs:
%   posterior        - 2D matrix representing the posterior distribution over actions.
%   n                - Number of anchor points to sample initially.
%   Rs               - Vector of possible values for the radial dimension (r).
%   Ths              - Vector of possible values for the angular dimension (theta).
%   dist_criterion   - Distance threshold for merging anchor points.
%
% Outputs:
%   r_anchors        - Sampled and merged radial coordinates of anchor points.
%   theta_anchors    - Sampled and merged angular coordinates of anchor points.
%   n                - Final number of anchor points after merging
   
% Create an array of indices for all possible actions (flattened posterior matrix).
   action_array=1:size(posterior,1)*size(posterior,2);

% Handle NaN values in the posterior by setting them to 0.
   temp_posterior=posterior;
   temp_posterior(isnan(temp_posterior(:)))=0;

% Sample 'n' unique actions from the action array based on the posterior weights.
   [l_ind]=datasample(action_array,n,'Replace',false,'Weights',(temp_posterior(:)));

% Convert the sampled linear indices to subscripts (row and column indices)
   [theta_temp_ind,r_temp_ind]=ind2sub(size(posterior),l_ind);
    
% Map the sampled indices to their corresponding radial (Rs) and angular (Ths) values.
   r_anchors=Rs(r_temp_ind);
   theta_anchors=Ths(theta_temp_ind);
    
% Create pairwise distance matrices for the sampled anchor points.
% xx and xx2 are column vectors of repeated radial and angular values.

    xx= repmat(r_anchors,n,1);
    xx=xx(:);
    xx2=repmat(theta_anchors,n,1);
    xx2=xx2(:);

% yy and yy2 are row vectors of repeated radial and angular values.
 
    yy=repmat(r_anchors',n,1);
    yy2= repmat(theta_anchors',n,1);

% Compute the difference between all pairs of anchor points.
    D1=[xx xx2]-[yy yy2];
% Normalize the differences by the range of Rs and Ths.
    rng_Rs=Rs(end)-Rs(1);
    rng_Ths=Ths(end)-Ths(1); 
    D_full= (D1)*[1/((rng_Rs)^2) 0; 0  1/((rng_Ths)^2)]*(D1');

% Compute the Euclidean distance between anchor points.
    D=sqrt(reshape(diag(D_full),[n n]));
    D(D==0)=nan; % Set diagonal (self-distances) to NaN to ignore them.
    
% Find the minimum distance and its indices in the distance matrix.
    [vmin,ind_min]= (min(D(:)));
    [r,c]=ind2sub(size(D),ind_min);

% Merge anchor points that are too close (distance < dist_criterion).
    while ((vmin<dist_criterion))

        % Choose which of the two close points to keep based on the higher posterior value.
        [~,ichs]=max( [posterior(theta_temp_ind(r),r_temp_ind(r)), posterior(theta_temp_ind(c),r_temp_ind(c))]);

        temm=[r,c];
        ichosen=temm(ichs); % Index of the chosen point to keep.
        
        
        % Remove the other point from the lists.
        choices=[1,2];
        not_chosen=choices(choices~=ichs);
        theta_temp_ind(temm(not_chosen))=[];
        r_temp_ind(temm(not_chosen))=[];
        r_anchors(temm(not_chosen))=[];
        theta_anchors(temm(not_chosen))=[];
     
        % Update the number of anchor points.
        n=size(r_anchors,2);

        % Recompute the distance matrix with the updated anchor points.        
        xx= repmat(r_anchors,n,1);
        xx=xx(:);
        xx2=repmat(theta_anchors,n,1);
        xx2=xx2(:);
        yy=repmat(r_anchors',n,1);
        yy2= repmat(theta_anchors',n,1);
        D1=[xx xx2]-[yy yy2];
        D_full= (D1)*[1/((rng_Rs)^2) 0; 0  1/((rng_Ths)^2)]*(D1');
        D=sqrt(reshape(diag(D_full),[n n]));
        D(D==0)=nan;
        
        % Find the new minimum distance and its indices.
        [vmin,ind_min]= (min(D(:)));
        [r,c]=ind2sub(size(D),ind_min);

    end


end