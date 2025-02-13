function L1= Likelihood(r_,thetas_,sigma,Rs,Thetas)

L1=zeros(size(Thetas,2),size(Rs,2));

L1_keypt=gpuArray(zeros(size(L1,1),size(L1,2),numel(r_)));

rgx=(Rs(end)-Rs(1));
rgy=(Thetas(end)-Thetas(1));

% express sigma in the normalized space.

cov=[sigma^2 0; 0 sigma^2];
[X,Y]=meshgrid(Rs,Thetas);


parfor key_pt=1:numel(r_)

    temp=zeros(size(L1));
    
    % x and y are the action values
    % of the ith (r,theta) key point on the loop
    x=r_(key_pt);
    y=thetas_(key_pt);
    
    for i=1:size(Y,1)
    
    Y_i=repmat(Y(i,1),1,size(X,2));
    d=[(x-X(i,:))/rgx; (y-Y_i)/rgy];

    D= exp(-0.5*(d)'*(inv(cov))*(d));
    temp(i,:)=diag(D);   
    end

    L1_keypt(:,:,key_pt)=temp;
    
end
L1=gather(sum(L1_keypt,3));

end


    
