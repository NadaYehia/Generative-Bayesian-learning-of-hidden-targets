function [r_anchors,theta_anchors,n]=anchors_prop_sampler_wnn_merge (posterior,n,Rs,Os,dist_criterion)

   
   action_array=[1:size(posterior,1)*size(posterior,2)];
   %handle the nans
   temp_posterior=posterior;
   temp_posterior(isnan(temp_posterior(:)))=0;
   [l_ind]=datasample(action_array,n,'Replace',false,'Weights',(temp_posterior(:)));
   
   [theta_temp_ind,r_temp_ind]=ind2sub(size(posterior),l_ind);
    
   r_anchors=Rs(r_temp_ind);
   theta_anchors=Os(theta_temp_ind);
    
 % xx=(nxn)x1 column vector of the first dimension in anchors: r
    xx= repmat(r_anchors,n,1);
    xx=xx(:);
    xx2=repmat(theta_anchors,n,1);
    xx2=xx2(:);

 % yy= (nn)x1 column vector of the first dimension in anchors: r
    yy=repmat(r_anchors',n,1);
    yy2= repmat(theta_anchors',n,1);

    D1=[xx xx2]-[yy yy2];
    rng_Rs=Rs(end)-Rs(1);
    rng_Os=Os(end)-Os(1); 
    D_= (D1)*[1/((rng_Rs)^2) 0; 0  1/((rng_Os)^2)]*(D1');

    D=sqrt(reshape(diag(D_),[n n]));
    D(D==0)=nan;
    
    
    [vmin,ind_min]= (min(D(:)));
    % D is nxn matrix. 
    [r,c]=ind2sub(size(D),ind_min);

    while ((vmin<dist_criterion))

        [~,ichs]=max( [posterior(theta_temp_ind(r),r_temp_ind(r)), posterior(theta_temp_ind(c),r_temp_ind(c))]);

        temm=[r,c];
        ichosen=temm(ichs);
        
        
        %
        choices=[1,2];
        not_chosen=choices(choices~=ichs);
        theta_temp_ind(temm(not_chosen))=[];
        r_temp_ind(temm(not_chosen))=[];

        r_anchors(temm(not_chosen))=[];
        theta_anchors(temm(not_chosen))=[];
     
        %
       
        n=size(r_anchors,2);

        % recalc D 
        xx= repmat(r_anchors,n,1);
        xx=xx(:);
        xx2=repmat(theta_anchors,n,1);
        xx2=xx2(:);
    
        yy=repmat(r_anchors',n,1);
        yy2= repmat(theta_anchors',n,1);
    
        D1=[xx xx2]-[yy yy2];
    
        D_= (D1)*[1/((rng_Rs)^2) 0; 0  1/((rng_Os)^2)]*(D1');
    
        D=sqrt(reshape(diag(D_),[n n]));
        D(D==0)=nan;
        
        
        [vmin,ind_min]= (min(D(:)));
         % D is nxn matrix. 
        [r,c]=ind2sub(size(D),ind_min);

    end


end