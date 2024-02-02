% this function reorder the anchors according to the list in tsp algorithm

function [mu_anchors, omega_anchors]= reorder_actions_anchors(mus,omegas,Gsol)

curr_node=1;
GsolE=table2array(Gsol.Edges);
nodes=size(GsolE,1);
mu_anchors=zeros(1,nodes);
omega_anchors=zeros(1,nodes);


    for ord_node=1:nodes
    
        [row_idx,~]=find(GsolE==curr_node);
        % the first item that has the current node on either side
        row_idx=row_idx(1);

        mu_anchors(ord_node)=mus(curr_node);
        omega_anchors(ord_node)=omegas(curr_node);
        
        temp=GsolE(row_idx,:);
        curr_node=temp(temp~=curr_node);
        GsolE(row_idx,:)=[];

    end


mu_anchors=[mu_anchors 0];
omega_anchors=[omega_anchors omega_anchors(end)];

end