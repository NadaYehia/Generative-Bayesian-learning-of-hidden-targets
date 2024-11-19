function L1= place_field_map_for_likelihood(x_,y_,sigma,As,Omegas,wrkrs)

L1=zeros(size(Omegas,2),size(As,2));
rgx=(As(end)-As(1));
rgy=(Omegas(end)-Omegas(1));

% express sigma in the normalized space: x pixels to distance in 
% normalized A,O space
sigma=sigma/numel(As);
[X,Y]=meshgrid(As,Omegas);

for key_pt=1:numel(x_)
    % create X and Y matrices where the rows in X are the vector in As
    % and the coloumn in Y are the vector in Omegas.
    
    
    % mux and muy are the action values
    % of the ith key point on the loop
    mux=x_(key_pt);
    muy=y_(key_pt);
    
    % temp place field like map from this keypoint   
    temp=(1/(2*pi*(sigma^2))).* exp(-(...
        ( ( (X-mux)./rgx ).^2 + ( (Y-muy)./rgy ).^2  )./(2*(sigma^2)) ));
    
    L1=L1+temp;
    temp=[];

end



  
end