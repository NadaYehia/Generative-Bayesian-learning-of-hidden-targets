% this function reorder the anchors according to the list in tsp algorithm

function [r_anchors, theta_anchors]= reorder_actions_anchors(rs,thetas,Gsol)

curr_node=1;
GsolE=table2array(Gsol.Edges);
nodes=size(GsolE,1);
r_anchors=zeros(1,nodes);
theta_anchors=zeros(1,nodes);


    for ord_node=1:nodes
    
        [row_idx,~]=find(GsolE==curr_node);
        % the first item that has the current node on either side
        row_idx=row_idx(1);

        r_anchors(ord_node)=rs(curr_node);
        theta_anchors(ord_node)=thetas(curr_node);
        
        temp=GsolE(row_idx,:);
        curr_node=temp(temp~=curr_node);
        GsolE(row_idx,:)=[];

    end


r_anchors=[r_anchors 0];
theta_anchors=[  theta_anchors 0];

end