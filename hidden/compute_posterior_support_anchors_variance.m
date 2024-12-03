% compute posterior support (as variance in its samples or entropy) to
% compare it with variance in executed anchors
% dd= size(posterior_support_omega_entropy,2);
% l=1;
% var_x_mu_temp1=zeros(1,(dd-(2*win)));
% var_x_mu_temp2=zeros(1,(dd-(2*win)));
% var_y_mu_temp=zeros(1,(dd-(2*win)));
% var_x_om_temp1=zeros(1,(dd-(2*win)));
% var_x_om_temp2=zeros(1,(dd-(2*win)));
% var_y_om_temp=zeros(1,(dd-(2*win)));
% 
% 
%    for sc=win+1:dd-win
%        var_x_mu_temp1(l)=(posterior_support_mu_var(sc));
%        var_x_mu_temp2(l)=(posterior_support_mu_entropy(sc));
%        
%        var_y_mu_temp(l)= var( cell2mat(mu_anchors_variance(sc-win:sc+win)) );
%        
%        var_x_om_temp1(l)=(posterior_support_omega_var(sc));
%        var_x_om_temp2(l)=(posterior_support_omega_entropy(sc));
% 
%        var_y_om_temp(l)= var( cell2mat(omega_anchors_variance(sc-win:sc+win)) );
% 
%        l=l+1;
%    end
% 
%    var_x_mu(agent,:)=var_x_mu_temp1;
%    entropy_x_mu(agent,:)=var_x_mu_temp2;
%    var_y_mu(agent,:)=var_y_mu_temp;
% 
%    var_x_om(agent,:)=var_x_om_temp1;
%    entropy_x_om(agent,:)=var_x_om_temp2;
%    var_y_om(agent,:)=var_y_om_temp;