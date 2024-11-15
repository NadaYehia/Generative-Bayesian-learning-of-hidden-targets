function [mu_anchors,omega_anchors,n]=anchors_prop_sampler_wnn_merge (posterior,n,As,Os,dist_criterion,lin_idx_bounds)

   l_ind= [];
   ancs=n;
   while(numel(l_ind)~=n)
    
    action_array=[1:size(posterior,1)*size(posterior,2)];
    [temp]=datasample(action_array,ancs,'Replace',false,'Weights',(posterior(:)));

    not_peaks=find(ismember(temp,lin_idx_bounds));
    temp(not_peaks)=[];
    l_ind=[l_ind,temp];
    ancs=n-numel(l_ind);

   end
   
    [omega_temp_ind,mu_temp_ind]=ind2sub(size(posterior),l_ind);
    
    mu_anchors=As(mu_temp_ind);

    omega_anchors=Os(omega_temp_ind);
    
    % xx=(nxn)x1 column vector of the first dimension in anchors: mu
    xx= repmat(mu_anchors,n,1);
    xx=xx(:);
    xx2=repmat(omega_anchors,n,1);
    xx2=xx2(:);

    % yy= (nn)x1 column vector of the first dimension in anchors: mu
    % repeat mu_anchors' vector 10 times and omega_anchors' vector also
    % 10x
    yy=repmat(mu_anchors',n,1);
    yy2= repmat(omega_anchors',n,1);

    D1=[xx xx2]-[yy yy2];
    rng_As=As(end)-As(1);
    rng_Os=Os(end)-Os(1); 
    D_= (D1)*[1/((rng_As)^2) 0; 0  1/((rng_Os)^2)]*(D1');

    D=sqrt(reshape(diag(D_),[n n]));
    D(D==0)=nan;
    
    
    [vmin,ind_min]= (min(D(:)));
    % D is nxn matrix. 
    [r,c]=ind2sub(size(D),ind_min);

    while ((vmin<dist_criterion))

        [~,ichs]=max( [posterior(omega_temp_ind(r),mu_temp_ind(r)), posterior(omega_temp_ind(c),mu_temp_ind(c))]);

        temm=[r,c];
        ichosen=temm(ichs);
        
        
        %
        choices=[1,2];
        not_chosen=choices(choices~=ichs);
        omega_temp_ind(temm(not_chosen))=[];
        mu_temp_ind(temm(not_chosen))=[];

        mu_anchors(temm(not_chosen))=[];
        omega_anchors(temm(not_chosen))=[];
     
        %
       
        n=size(mu_anchors,2);

        % recalc D 
        xx= repmat(mu_anchors,n,1);
        xx=xx(:);
        xx2=repmat(omega_anchors,n,1);
        xx2=xx2(:);
    
        yy=repmat(mu_anchors',n,1);
        yy2= repmat(omega_anchors',n,1);
    
        D1=[xx xx2]-[yy yy2];
    
        D_= (D1)*[1/((rng_As)^2) 0; 0  1/((rng_Os)^2)]*(D1');
    
        D=sqrt(reshape(diag(D_),[n n]));
        D(D==0)=nan;
        
        
        [vmin,ind_min]= (min(D(:)));
         % D is nxn matrix. 
        [r,c]=ind2sub(size(D),ind_min);

    end


end