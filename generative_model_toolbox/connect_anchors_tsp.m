function [Gsol, r_anchors, theta_anchors]= connect_anchors_tsp(r_anchors,theta_anchors,nStops,Rs,Ths)

% This function connects the list of r and theta anchors using the Traveling Salesman Problem (TSP).
% It Formulates the problem as an integer linear program (ILP). Ensuring each stop is visited exactly
% twice (once as a start and once as an end). Detecting subtours (loops) in the
% solution, which is a common issue in TSP formulations.

% Input: list of N anchors in R, theta space
%        nstops: number of stops to visit (N anchors + the start at Home).
%        Rs,Ths: R and theta domain values. 

% Output: Gsol: connected graph between every anchor and its closest
% anchor to visit, 
%         r_anchors,theta_anchors: list of anchors with Home anchor
%         appended at the start.


%% Generate all possible pairs of stops using combinatorial indexing
idxs = nchoosek(1:nStops,2);
% Extract radii (r) and angles (theta) for the starting points of each pair
r_anchors_ptS=r_anchors(idxs(:,1));
theta_anchors_ptS=theta_anchors(idxs(:,1));
% Extract radii (r) and angles (theta) for the ending points of each pair
r_anchors_ptE=r_anchors(idxs(:,2));
theta_anchors_ptE=theta_anchors(idxs(:,2));

% Loop through each pair of points to calculate the scaled Euclidean distance
for k=1:size(idxs,1)
    % Compute the squared difference in r and theta coordinates for the k-th pair
    delta_r= (r_anchors_ptS(k) -r_anchors_ptE(k))^2;
    delta_theta= (theta_anchors_ptS(k)- theta_anchors_ptE(k))^2;
    % Compute the squared scaling factor for r and theta
    scaleR_sq=( Rs(end)-Rs(1) )^2;
    scaleTh_sq=(Ths(end)-Ths(1))^2;
    % Calculate the scaled Euclidean distance for the k-th pair
    dist(k)= sqrt( (delta_r/scaleR_sq) + (delta_theta/scaleTh_sq)     );

end
lendist = length(dist);
%%
%Create the linear constraints that each stop has two associated trips,
% because there must be a trip to each stop and a trip departing each stop.

%  Allocate a sparse matrix nStops x length(idxs), 
% with space for nStops*(nStops-1) non-zero elements   
Aeq = spalloc(nStops,length(idxs),nStops*(nStops-1)); 

% Construct the equality constraint matrix (Aeq)
for ii = 1:nStops
    whichIdxs = (idxs == ii); % Find the trips that include stop ii
    whichIdxs = sparse(sum(whichIdxs,2)); % Include trips where ii is at either end
    Aeq(ii,:) = whichIdxs'; % Each row of Aeq ensures that stop ii is visited exactly twice
end

% Define the right-hand side of the equality constraints (beq)
% Each stop must be visited exactly twice (once as a start and once as an end)
beq = 2*ones(nStops,1);

% Define integer decision variables (intcon)
% All variables are integers since we are solving an ILP problem
intcon = 1:lendist;

% Define lower and upper bounds for the decision variables
% Each decision variable (x_tsp) is binary (0 or 1)
lb = zeros(lendist,1);
ub = ones(lendist,1);

% Solve the integer linear programming problem
% dist: Objective function (minimize total distance)
% intcon: Indices of integer variables
% Aeq, beq: Equality constraints (each stop visited exactly twice)
% lb, ub: Lower and upper bounds for decision variables
opts = optimoptions('intlinprog','Display','off');
[x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,[],[],Aeq,beq,lb,ub,opts);

% Round the solution to binary values (0 or 1) and convert to logical
x_tsp = logical(round(x_tsp));

% Construct a graph (Gsol) from the selected trips (edges)
% idxs(x_tsp, 1) and idxs(x_tsp, 2) are the start and end stops of the selected trips
Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2));

%%  This section of the code eliminates subtours, inner loops, and makes sure a single tour visits all stops

A = spalloc(0,lendist,0); % Allocate a sparse linear inequality constraint matrix (initially empty)
% Define the right-hand side of the inequality constraints (b)
% Initially empty, as no inequality constraints are used here
b = []; 
% Find connected components (subtours) in the solution graph (Gsol)
tourIdxs = conncomp(Gsol);
numtours = max(tourIdxs); % number of subtours

while numtours > 1 % Repeat until there is just one subtour

    % Add new rows to the inequality constraint matrix (A) and right-hand side vector (b)
    % to accommodate subtour elimination constraints
    b = [b;zeros(numtours,1)]; 
    A = [A;spalloc(numtours,lendist,nStops)]; 
    
    % Iterate over each subtour to add constraints
    for ii = 1:numtours

       % Get the current row index for the new constraint
        rowIdx = size(A,1) + 1; 
       % Find the stops that belong to the current subtour
        subTourIdx = find(tourIdxs == ii); 

%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        for jj = 1:length(variations)
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
            A(rowIdx,whichVar) = 1;
        end
        b(rowIdx) = length(subTourIdx) - 1; % One less trip than subtour stops
    end

    % Re-optimize the problem with the new constraints
    [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,A,b,Aeq,beq,lb,ub,opts);
    x_tsp = logical(round(x_tsp));
    % Reconstruct the solution graph (Gsol) using the updated trips
    Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); 
 
    % Recompute the connected components (subtours) in the updated graph
    tourIdxs = conncomp(Gsol);
    % Update the number of subtours
    numtours = max(tourIdxs); 

 end

end