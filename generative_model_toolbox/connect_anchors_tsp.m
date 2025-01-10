% connect action anchors using Travelling salesman problem formulation
% Created by Nada Abdelrahman, adapted from Matlab optimization toolbox


function [Gsol, r_anchors, theta_anchors]= connect_anchors_tsp(r_anchors_i,theta_anchors_i,nStops,Rs,Ths)
     
    r_anchors=r_anchors_i;
    theta_anchors=theta_anchors_i;
    
    idxs = nchoosek(1:nStops,2);
    r_anchors_ptS=r_anchors(idxs(:,1));
    theta_anchors_ptS=theta_anchors(idxs(:,1));

    r_anchors_ptE=r_anchors(idxs(:,2));
    theta_anchors_ptE=theta_anchors(idxs(:,2));

    for k=1:size(idxs,1)
        delta_r= (r_anchors_ptS(k) -r_anchors_ptE(k))^2;
        delta_theta= (theta_anchors_ptS(k)- theta_anchors_ptE(k))^2;
        scaleR_sq=( Rs(end)-Rs(1) )^2;
        scaleTh_sq=(Ths(end)-Ths(1))^2;
        dist(k)= sqrt( (delta_r/scaleR_sq) + (delta_theta/scaleTh_sq)     );
    
    end

    lendist = length(dist);


    %Create the linear constraints that each stop has two associated trips,
    % because there must be a trip to each stop and a trip departing each stop.

        Aeq = spalloc(nStops,length(idxs),nStops*(nStops-1)); % Allocate a sparse matrix
        for ii = 1:nStops
            whichIdxs = (idxs == ii); % Find the trips that include stop ii
            whichIdxs = sparse(sum(whichIdxs,2)); % Include trips where ii is at either end
            Aeq(ii,:) = whichIdxs'; % Include in the constraint matrix
        end
        beq = 2*ones(nStops,1);


        intcon = 1:lendist;
        lb = zeros(lendist,1);
        ub = ones(lendist,1);

        opts = optimoptions('intlinprog','Display','off');
       [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,[],[],Aeq,beq,lb,ub,opts);

       x_tsp = logical(round(x_tsp));
       Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2));


      A = spalloc(0,lendist,0); % Allocate a sparse linear inequality constraint matrix
      b = [];

      tourIdxs = conncomp(Gsol);
      numtours = max(tourIdxs); % number of subtours
     
      while numtours > 1 % Repeat until there is just one subtour
        % Add the subtour constraints
        b = [b;zeros(numtours,1)]; % allocate b
        A = [A;spalloc(numtours,lendist,nStops)]; % A guess at how many nonzeros to allocate
        for ii = 1:numtours
            rowIdx = size(A,1) + 1; % Counter for indexing
            subTourIdx = find(tourIdxs == ii); % Extract the current subtour
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
    
        % Try to optimize again
        [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,A,b,Aeq,beq,lb,ub,opts);
        x_tsp = logical(round(x_tsp));
%         Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2),[],numnodes(G));
        Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); % Also works in most cases
     
        % How many subtours this time?
        tourIdxs = conncomp(Gsol);
        numtours = max(tourIdxs); % number of subtours
       end

end