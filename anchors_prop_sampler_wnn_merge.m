function [mu_anchors,omega_anchors,n]=anchors_prop_sampler_wnn_merge (posterior,n,As,Os,dist_criterion,r_bounds,c_bounds)

    action_array=[1:size(posterior,1)*size(posterior,2)];
    l_ind= datasample(action_array,n,'Replace',false,'Weights',(posterior(:)));
   
    [omega_temp_ind,mu_temp_ind]=ind2sub(size(posterior),l_ind);
    
    mu_anchors=As(mu_temp_ind);

    if (max(omega_temp_ind)>size(Os,2))
        debugg=1;
    end
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

    D_= (D1)*[1/(std(As)^2) 0; 0  1/(std(Os)^2)]*(D1');

    D=sqrt(reshape(diag(D_),[n n]));
    D(D==0)=nan;
    
    
    [vmin,ind_min]= (min(D(:)));
    % D is nxn matrix. 
    [r,c]=ind2sub(size(D),ind_min);

    while ((vmin<dist_criterion))

        [~,ichs]=max( [posterior(omega_temp_ind(r),mu_temp_ind(r)), posterior(omega_temp_ind(c),mu_temp_ind(c))]);

        temm=[r,c];
        ichosen=temm(ichs);
        mu_anchors_copy(r)=(mu_anchors(ichosen));
        mu_anchors_copy(c)= (mu_anchors(ichosen));

        omega_anchors_copy(r)=omega_anchors(ichosen);
        omega_anchors_copy(c)=omega_anchors(ichosen);


       
        mu_anchors(r)=mu_anchors_copy(r);
        mu_anchors(c)=mu_anchors_copy(c);
        mu_anchors_copy=[];

        omega_anchors(r)=omega_anchors_copy(r);
        omega_anchors(c)=omega_anchors_copy(c);
        omega_anchors_copy=[];
        cmbd_anchors=[mu_anchors' omega_anchors'];

        unique_anchrs=unique(cmbd_anchors,'rows');
        mu_anchors=(unique_anchrs(:,1))';
        omega_anchors=(unique_anchrs(:,2))';
        n=size(mu_anchors,2);

        % recalc D 
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
    
        D_= (D1)*[1/(std(As)^2) 0; 0  1/(std(Os)^2)]*(D1');
    
        D=sqrt(reshape(diag(D_),[n n]));
        D(D==0)=nan;
        
        
        [vmin,ind_min]= (min(D(:)));
         % D is nxn matrix. 
        [r,c]=ind2sub(size(D),ind_min);

    end


end