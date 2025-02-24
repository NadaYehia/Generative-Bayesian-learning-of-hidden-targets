function L1= Likelihood(rs_,thetas_,sigma,Rs,Thetas)
% This function represents the agent's internal model of the environment.
% It computes the object [L(outcome | r,t, R,Theta)], which is the likelihood of a an outcome being a reward (outcome=1)
% given the set of radial (r) and angular (theta) actions points along the
% executed path and if every location (R,theta) in the arena (posterior grid) 
% could be the hidden target location.
% This object can be used as is in the Bayesian update step 
% if the current outcome is reward (1) or it can be inverted (1-L) in which
% case it represents the likelihood of an outcome being no reward (0) 
% given the same path and the (R,theta) grid of possible target locations.

% Inputs:
%   rs_: Radial coordinates of the executed points along the trajectory.
%   thetas_: Angular coordinates of the executed points along the trajectory.
%   sigma: Standard deviation for the Gaussian distribution.
%   Rs: possible values in the radii domain.
%   Thetas: possible values in the angular domain.
% Output:
%   L1: Likelihood matrix computed over the grid (Rs, Thetas).


%%

% Initialize the likelihood matrix L1 with zeros
L1=zeros(size(Thetas,2),size(Rs,2));

% Initialize a 3D matrix to store likelihood contributions from each key point
L1_keypt=(zeros(size(L1,1),size(L1,2),numel(rs_)));

% Compute the range of radial (rgx) and angular (rgy) coordinates
rgx=(Rs(end)-Rs(1));
rgy=(Thetas(end)-Thetas(1));

% Create a meshgrid of radial (Rs) and angular (Thetas) coordinates
[X,Y]=meshgrid(Rs,Thetas);

% Parallel loop over all key points
parfor key_pt=1:numel(rs_)
   
    % Initialize a temporary matrix to store the likelihood for the current key point
    temp=zeros(size(L1));
    
    % x and y are the action values
    % of the ith (r,theta) key point on the executed loop
    x=rs_(key_pt);
    y=thetas_(key_pt);

    % Compute the normalized differences between the key point and the grid
    dX=(x-X)/rgx;
    dY=(y-Y)/rgy;

    % Compute the squared distance (scaled by 1/sigma^2)
    sq_dist = ((1/(sigma^2)).*dX.^2 +...
             (1/(sigma^2)) .* dY.^2);

    % Compute the normalization constant for the Gaussian distribution
    N_c=1/(sqrt(2*pi)*(sigma^2));

    % Compute the Gaussian likelihood for the current key point
    temp= N_c*exp(-0.5*sq_dist);
    
    % Store the likelihood contribution of the current key point
    L1_keypt(:,:,key_pt)=temp;
    
end

% Sum the likelihood contributions from all key points
L1=(sum(L1_keypt,3));

end


    
