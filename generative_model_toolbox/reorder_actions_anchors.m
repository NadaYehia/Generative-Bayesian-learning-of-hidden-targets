function [r_anchors, theta_anchors]= reorder_actions_anchors(rs,thetas,Gsol)

% This function reorders the radial and angular coordinates of nodes based on a graph traversal
% It reconstruct a path or tour from the Traveling Salesman Problem graph solution 
% ans ensure the coordinates are ordered according to the sequence of nodes visited in the graph.

% Reorders the radial (r) and angular (theta) coordinates of nodes based on the graph traversal.
% Inputs:
%   rs: Radial coordinates of all nodes.
%   thetas: Angular coordinates of all nodes.
%   Gsol: Graph object representing the solution (edges connecting nodes).
% Outputs:
%   r_anchors: Reordered radial coordinates.
%   theta_anchors: Reordered angular coordinates.

%%
% Start traversal from the first node
curr_node=1;
% Convert graph edges to an array for easier manipulation
GsolE=table2array(Gsol.Edges);
% Determine the number of nodes (edges) in the graph
nodes=size(GsolE,1);
% Initialize arrays to store reordered radial and angular coordinates
r_anchors=zeros(1,nodes);
theta_anchors=zeros(1,nodes);

% Traverse the graph and reorder the coordinates
for ord_node=1:nodes
    % Find the row index of the edge that contains the current node
    [row_idx,~]=find(GsolE==curr_node);

    % Use the first occurrence of the current node in the edge list
    row_idx=row_idx(1);

    % Store the radial and angular coordinates of the current node
    r_anchors(ord_node)=rs(curr_node);
    theta_anchors(ord_node)=thetas(curr_node);

    % Get the connected node (the other node in the edge)
    temp=GsolE(row_idx,:);
    curr_node=temp(temp~=curr_node); % Update current node to the connected node
  
    % Remove the traversed edge from the edge list to avoid revisiting
    GsolE(row_idx,:)=[];

end

% Append a zero at the end of the reordered coordinates
% This represent a return to the starting point (Home).
r_anchors=[r_anchors 0];
theta_anchors=[  theta_anchors 0];

end