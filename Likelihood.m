function L1= Likelihood(r_,thetas_,sigma,Rs,Thetas)

L1=zeros(size(Thetas,2),size(Rs,2));
rgx=(Rs(end)-Rs(1));
rgy=(Thetas(end)-Thetas(1));

% express sigma in the normalized space.
sigma=sigma/numel(Rs);

[X,Y]=meshgrid(Rs,Thetas);

for key_pt=1:numel(r_)
   
    % x and y are the action values
    % of the ith (r,theta) key point on the loop
    x=r_(key_pt);
    y=thetas_(key_pt);
    
    % bin the x and y to the closest bins in Rs and Thetas

    [~,idx_binX]=(min(abs(x-Rs)));
    [~,idx_binY]=(min(abs(y-Thetas)));
    
    x_binned=Rs(idx_binX);
    y_binned=Thetas(idx_binY);

    % 2D Gaussian centered at x_binned, y_binned  
    temp=(1/(2*pi*(sigma^2))).* exp(-(...
        ( ( (X-x_binned)./rgx ).^2 + ( (Y-y_binned)./rgy ).^2  )./(2*(sigma^2)) ));
    
    L1=L1+temp;
    temp=[];

end



  
end