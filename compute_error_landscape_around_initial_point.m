
%explore the error in increase direction/decrease for every parameter
% 
function loss_grid_near_p0=compute_error_landscape_around_initial_point(kd,r0,theta0,...
             n,arena,ka,no_anchors,w2,rg_r,optimal_para)

q=numel(r0)+numel(theta0)+numel(kd);
lb=zeros(1,q);
ub=zeros(1,q);

lb(1:no_anchors-1)=-inf;
% curvature lower bound on the first anchor
lb(1)=-( pi-(theta0(2)) );
% curvature lower bound on the first anchor
lb_r_th=zeros(1,2*no_anchors); %radius can't be negative 
lb(no_anchors:end)=lb_r_th;

%%
ub(1:no_anchors-1)=inf;

%curvature upper bound on the first anchor
ub(1)=theta0(2); 
%curvature upper bound on first anchor
ub_r_th(1:no_anchors)=repmat(rg_r,1,no_anchors);

ub_r_th(no_anchors+1:2*no_anchors)=pi;

ub(no_anchors:end)=ub_r_th;

d=1;
initial_para=[kd,r0,theta0];
loss_grid_near_p0=zeros(numel(initial_para),n+1);
for p=1:numel(initial_para)

    % explore the error along a straight line for every dimension around
    % the optimal_para(p0)

    dim_p=linspace(initial_para(p)-d,initial_para(p)+d,n);

  
    % call the loss function for every dimension, 
    %compute the loss function at every point of the n points

    for i=1:n

        if(dim_p(i)<lb(p) || dim_p(i)>ub(p))
            loss_grid_near_p0(p,i)=200;
        
        else
            Para=initial_para;
            Para(p)=dim_p(i);
            total_cost=my_path_length(Para,arena,ka,no_anchors,w2);
            loss_grid_near_p0(p,i)=total_cost;
        end
        
    end
    [~,closest_bin]=min(abs(optimal_para(p)-dim_p));
    optimal_coord=closest_bin;
    loss_grid_near_p0(p,n+1)=optimal_coord;

end