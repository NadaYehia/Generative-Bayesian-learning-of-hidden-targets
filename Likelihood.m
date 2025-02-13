function L1= Likelihood(r_,thetas_,sigma,Rs,Thetas)

L1=zeros(size(Thetas,2),size(Rs,2));

L1_keypt=(zeros(size(L1,1),size(L1,2),numel(r_)));

rgx=(Rs(end)-Rs(1));
rgy=(Thetas(end)-Thetas(1));

% express sigma in the normalized space.

[X,Y]=meshgrid(Rs,Thetas);


parfor key_pt=1:numel(r_)

    temp=zeros(size(L1));
    
    % x and y are the action values
    % of the ith (r,theta) key point on the loop
    x=r_(key_pt);
    y=thetas_(key_pt);
    dX=(x-X)/rgx;
    dY=(y-Y)/rgy;
    sq_dist = ((1/(sigma^2)).*dX.^2 +...
             (1/(sigma^2)) .* dY.^2);

    N_c=1/(sqrt(2*pi)*(sigma^2));
    temp= N_c*exp(-0.5*sq_dist);
    
    L1_keypt(:,:,key_pt)=temp;
    
end
L1=(sum(L1_keypt,3));

end


    
