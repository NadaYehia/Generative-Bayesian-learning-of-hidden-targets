close all
clear all
clc
intercept= 'anypt';

sig_spd= 10;
sig_hd= 0.25;
restr_lvl=[0];
sigma_ridge=[20];

for r_level=1:size(restr_lvl,2)
    anchors=[10:15];
    Merge_dist=[1:2:10].*sigma_ridge;

    
    arena.dims = [-400 400 0 600];
    blocks = [60 150];

    arena.x = arena.dims([1 2 2 1]);
    arena.y = arena.dims([3 3 4 4]);

    % target A specs
    target.cntr = [10 250];
    target.width = [50 50];
    target.x = [target.cntr(1)-target.width(1) target.cntr(1)+target.width(1) target.cntr(1)+target.width(1) target.cntr(1)-target.width(1)];
    target.y = [target.cntr(2)-target.width(2) target.cntr(2)-target.width(2) target.cntr(2)+target.width(2) target.cntr(2)+target.width(2)];

    % target B specs
    target2.cntr = [-250 250];
    target2.width = [50 50];
    target2.x = [target2.cntr(1)-target2.width(1) target2.cntr(1)+target2.width(1) target2.cntr(1)+target2.width(1) target2.cntr(1)-target2.width(1)];
    target2.y = [target2.cntr(2)-target2.width(2) target2.cntr(2)-target2.width(2) target2.cntr(2)+target2.width(2) target2.cntr(2)+target2.width(2)];
 
%     figure(1);
%     patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
%     patch(target.x,target.y,[0.85 0.9 0.9]);
%     patch(target2.x,target2.y,[0.9 0.9 0.85]);
%     figure(2);
    
for an=1:size(anchors,2)
    anchors_no_int=anchors(an);

    for dc=1:size(Merge_dist,2)
        
        
        dist_criterion=func_sigma_ridge_dist(1,sigma_ridge);
        win=1;

        var_x_mu=zeros(50,(blocks(2)-(2*win)));
        entropy_x_mu=zeros(50,(blocks(2)-(2*win)));
        var_y_mu=zeros(50,(blocks(2)-(2*win)));
        var_x_om=zeros(50,(blocks(2)-(2*win)));
        entropy_x_om=zeros(50,(blocks(2)-(2*win)));
        var_y_om=zeros(50,(blocks(2)-(2*win)));

     parfor q=1:50
        
        tic

        var_x_mu_temp1=zeros(1,(blocks(2)-(2*win)));
        var_x_mu_temp2=zeros(1,(blocks(2)-(2*win)));
        var_y_mu_temp=zeros(1,(blocks(2)-(2*win)));
        var_x_om_temp1=zeros(1,(blocks(2)-(2*win)));
        var_x_om_temp2=zeros(1,(blocks(2)-(2*win)));
        var_y_om_temp=zeros(1,(blocks(2)-(2*win)));

        mu_anchors_variance={};
        omega_anchors_variance={};
        anchors_no_variance={};

        posterior_support_mu_var=zeros(1,sum(blocks));
        posterior_support_omega_var=zeros(1,sum(blocks));
         
        posterior_support_mu_entropy=zeros(1,sum(blocks));
        posterior_support_omega_entropy=zeros(1,sum(blocks));
 
      
        initial_ancs=10;
        
        prior_reset=0;
        expected_reward_rate=[];
        k_r=1;

        anchors_no=[];
        rndm_speed_seed=100+randperm(900,1);
    
    
          mu_spd=zeros(1,sum(blocks));
          om_hd=zeros(1,sum(blocks));
          om_main=zeros(1,sum(blocks));

    
         % a running trial
         target_hit = zeros(1,sum(blocks)); 
         hit_time = ones(1,sum(blocks)); 
         As=[1:2:1000];

         speed_step=max(As)/numel(As);

         Os=[-pi/2:2*pi/1000:pi/2];
         
         prior=ones(size(Os,2),size(As,2));
        
         % mask the prior with the action boundaries
         [mu_boundary,omega_boundary]=find_actions_bounds(arena);
         % given these values create a binary mask 
         [bw_boundary,r_bounds,c_bounds]=convert_poly_to_mask(mu_boundary,omega_boundary,size(prior),As,Os);
        

         prior=prior.*bw_boundary;
         prior=prior./(sum(sum(prior)));
         
         pos_x=[];
         pos_y=[];

         for k=1:blocks(2)
             
            
             if(k==1) 

               
               lin_idx_bounds=sub2ind(size(prior),r_bounds,c_bounds);
            
               % add the indices for the first row, last row,, first column
               ind_first_row= sub2ind(size(prior),repmat(1,size(prior,2),1),(1:size(prior,2))');
               ind_last_row= sub2ind(size(prior),repmat(501,size(prior,2),1),(1:size(prior,2))');
            
            
               lin_idx_bounds=[lin_idx_bounds, [1:size(prior,1)], ind_first_row', ind_last_row'];
               
               l_ind=[];
               ancs=initial_ancs;

               while(numel(l_ind)~=initial_ancs)
                
                action_array=[1:size(prior,1)*size(prior,2)];
                [temp]=datasample(action_array,ancs,'Replace',false,'Weights',(prior(:)));
           
                not_peaks=find(ismember(temp,lin_idx_bounds));
                temp(not_peaks)=[];
                l_ind=[l_ind,temp];
                ancs=initial_ancs-numel(l_ind);
        
               end
     
               [omega_temp_ind,mu_temp_ind]=ind2sub(size(prior),l_ind);
    
                mu_anchors=As(mu_temp_ind);

    
                 omega_anchors=Os(omega_temp_ind);
                                                 
                 [~,min_speed_anchor]=min(mu_anchors);
                 initial_hd=omega_anchors(min_speed_anchor);

                 % apply tsp on the anchors points
                 Gsol=connect_anchors_tsp([0 mu_anchors]',[ initial_hd omega_anchors]',initial_ancs+1,As,Os);
                 
                 % reorder the anchors according to the TSP
                 % list order
                 [mu_anchors,omega_anchors]=reorder_actions_anchors([0 mu_anchors],[ initial_hd omega_anchors],Gsol);

                % interpolate between the ordered anchors actions
                [mus_,omegas_,x_,y_]= connect_actions_with_speed_profile(mu_anchors,omega_anchors,sigma_ridge,speed_step);

                % omega_anchors(2:end-1), mu_anchors(2:end-1)
                om_main(k)=mean(omegas_);
                mu_spd(k)=mean(mus_);      

           
            
             else
                 % convert subset of speeds and omegas to x_ and y_ anchor points. 
                om_main(k)=mean(omegas_);
                mu_spd(k)=mean(mus_);   
               
             end

           

            pos_x=[ x_ ];
            pos_y=[ y_];

            for t=2:size(pos_x,2)

                if pos_x(t)<arena.dims(1)
                    pos_x(t)=arena.dims(1);                
                elseif pos_x(t)>arena.dims(2)
                    pos_x(t)=arena.dims(2);
                end

                if pos_y(t)<arena.dims(3)
                    pos_y(t)=arena.dims(3);                
                elseif pos_y(t)>arena.dims(4)
                    pos_y(t)=arena.dims(4);
                end

%                  if k>blocks(1)
%                     curr_target = target2;
%                 else
                    curr_target = target2;            
%                 end

                   
                if pos_y(t)>curr_target.cntr(2)-curr_target.width(2) & pos_y(t)<curr_target.cntr(2)+curr_target.width(2) & pos_x(t)>curr_target.cntr(1)-curr_target.width(1) & pos_x(t)<curr_target.cntr(1)+curr_target.width(1)
                    target_hit(k) = 1;
                    if hit_time(k)==1
                        hit_time(k) = t;
                    end
                end


            end

%            if(numel(mu_anchors)==3)     
% 
%                 if (target_hit(k)==1)
%                     anchors_x_y_rewarded(q,k,:)=[x_(numel(x_)/2),y_(numel(y_)/2)];
%                 else
%                     anchors_x_y_non_rewarded(q,k,:)=[x_(numel(x_)/2),y_(numel(y_)/2)];
%                 end
%            end
    %% Bayes update for the control parameters
    
       
             L1=Dist_from_ridge_prob_conv(mus_,omegas_,sigma_ridge,As,Os);
              
             L1= (L1-min(L1(:))) ./ ( max(L1(:))- min(L1(:)) );
         
%                if (prior_reset==0) 
%                 reward_linfit= polyfit([1:k],cumsum(target_hit(1:k)),1);
%                 expected_reward_rate(end+1)=reward_linfit(1); % slope of the cumsum at this point
% 
%                else
% 
%                    reward_linfit= polyfit([k_r:k],cumsum(target_hit(k_r:k)),1);
%                    expected_reward_rate(end+1)=reward_linfit(1); % slope of the cumsum at this point
%                end
         
                        if (target_hit(k)==1)
                            
                             posterior=Bayes_estimate(L1,1,prior);
                             posterior=posterior./( sum(sum(posterior)) );
                             
                             [lind]= datasample(action_array,100,'Replace',false,'Weights',(posterior(:)));
                             [omega_ind,mu_ind]=ind2sub(size(posterior),lind);

                             posterior_support_mu_var(k)= var(As(mu_ind));
                             posterior_support_omega_var(k)=var(Os(omega_ind));

                             posterior_support_omega_entropy(k)=my_entropy(posterior);
                             posterior_support_mu_entropy(k)=my_entropy(posterior);

                             [mu_anchors,omega_anchors,anchors_no(k)]=anchors_sampler_nn_merge (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
%                              [mu_anchors,omega_anchors,anchors_no(k)]=anchors_sampler_MuliPeakPost (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
                             
                             mu_anchors_variance{k}=(mu_anchors);
                             omega_anchors_variance{k}=(omega_anchors);
                             anchors_no_variance{k}=anchors_no(k);

                             [~,min_speed_anchor]=min(mu_anchors);
                             initial_hd=omega_anchors(min_speed_anchor);
                             
                             if(anchors_no(k)>1)

                                 % apply tsp on the anchors points
                                 Gsol=connect_anchors_tsp([0 mu_anchors]',[ initial_hd omega_anchors]',anchors_no(k)+1,As,Os);
                                 % reorder the anchors according to the TSP
                                 % list order
                                 [mu_anchors,omega_anchors]=reorder_actions_anchors([0 mu_anchors],[ initial_hd omega_anchors],Gsol);

                                  % interpolate between the ordered anchors actions
                                  [mus_,omegas_,x_,y_]= connect_actions_with_speed_profile(mu_anchors,omega_anchors,sigma_ridge,speed_step);
                             else
                                 mu_anchors=[0 mu_anchors 0];
                                 omega_anchors=[omega_anchors, omega_anchors, omega_anchors];
                                 [mus_,omegas_,x_,y_]= connect_actions_with_speed_profile(mu_anchors,omega_anchors,sigma_ridge,speed_step);
                                 

                             end
                            
                             

                            else
                             

                             posterior=Bayes_estimate(L1,0,prior);
                             posterior=posterior./( sum(sum(posterior)) );
                            
                             [lind]= datasample(action_array,100,'Replace',false,'Weights',(posterior(:)));
                             [omega_ind,mu_ind]=ind2sub(size(posterior),lind);
                            
                             posterior_support_mu_var(k)= var(As(mu_ind));
                             posterior_support_omega_var(k)=var(Os(omega_ind));


                             posterior_support_omega_entropy(k)=my_entropy(posterior);
                             posterior_support_mu_entropy(k)=my_entropy(posterior);

                             [mu_anchors,omega_anchors,anchors_no(k)]=anchors_sampler_nn_merge (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
                             
%                              [mu_anchors,omega_anchors,anchors_no(k)]=anchors_sampler_MuliPeakPost (posterior,initial_ancs,As,Os,dist_criterion,r_bounds,c_bounds);
                             
                             mu_anchors_variance{k}=(mu_anchors);
                             omega_anchors_variance{k}=(omega_anchors);
                             anchors_no_variance{k}=anchors_no(k);
                             
                             
                             
                             [~,min_speed_anchor]=min(mu_anchors);
                             initial_hd=omega_anchors(min_speed_anchor);

                            if(anchors_no(k)>1)

                                 % apply tsp on the anchors points
                                 Gsol=connect_anchors_tsp([0 mu_anchors]',[initial_hd omega_anchors]',anchors_no(k)+1,As,Os);
    
                                 % reorder the anchors according to the TSP
                                 % list order
    
                                 [mu_anchors,omega_anchors]=reorder_actions_anchors([0 mu_anchors],[ initial_hd omega_anchors],Gsol);

                            
                             % interpolate between the ordered anchors actions

                             [mus_,omegas_,x_,y_]= connect_actions_with_speed_profile(mu_anchors,omega_anchors,sigma_ridge,speed_step);
                            else
                                 mu_anchors=[0 mu_anchors 0];
                                 omega_anchors=[omega_anchors, omega_anchors, omega_anchors];
                                 [mus_,omegas_,x_,y_]= connect_actions_with_speed_profile(mu_anchors,omega_anchors,sigma_ridge,speed_step);

                            end

                        end
    
                prior= posterior;
           
%       figure(2);
%       imagesc(As,Os,prior);
%       
%       hold on, scatter(mu_anchors,omega_anchors,30,'r','filled');
%       hold off;
%       figure(1);       
%       hold on, plot(pos_x,pos_y,'LineWidth',0.5,'Color',[1 0.1 0.8 0.5]);
%       hold on,scatter(pos_x,pos_y,10,[1:numel(pos_y)],'filled');
%       
%       clf;
%       patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
%       patch(target.x,target.y,[0.85 0.9 0.9]);
%       patch(target2.x,target2.y,[0.9 0.9 0.85]);
    
    pos_x=[]; pos_y=[];
         end
    
    mu_pop_avg(q,:,dc,an) = mu_spd;
    hd_pop_avg(q,:,dc,an)=om_main;
    corr_pop_avg(q,:,dc,an) = target_hit;
    anchors_no_pop(q,:,dc,an)=anchors_no;
    
    mus_anchors_all{q}=mu_anchors_variance;
    oms_anchors_all{q}=omega_anchors_variance;
         

   
   l=1;
   for sc=win+1:k-win
       var_x_mu_temp1(l)=(posterior_support_mu_var(sc));
       var_x_mu_temp2(l)=(posterior_support_mu_entropy(sc));
       
       var_y_mu_temp(l)= var( cell2mat(mu_anchors_variance(sc-win:sc+win)) );
       
       var_x_om_temp1(l)=(posterior_support_omega_var(sc));
       var_x_om_temp2(l)=(posterior_support_omega_entropy(sc));

       var_y_om_temp(l)= var( cell2mat(omega_anchors_variance(sc-win:sc+win)) );

       l=l+1;
   end

   var_x_mu(q,:)=var_x_mu_temp1;
   entropy_x_mu(q,:)=var_x_mu_temp2;

   var_y_mu(q,:)=var_y_mu_temp;

   var_x_om(q,:)=var_x_om_temp1;
   entropy_x_om(q,:)=var_x_om_temp2;

   var_y_om(q,:)=var_y_om_temp;
   
   toc
    end
    scale = 1;
figure;
hold on, shadedErrorBar(1:blocks(2), mean(corr_pop_avg(:,1:blocks(2))) ,std( corr_pop_avg(:,1:blocks(2)))./sqrt(size(corr_pop_avg,1) )./scale,{'color','g'}); hold on;
figure;
hold on, shadedErrorBar(1:blocks(2), mean(mu_pop_avg(:,1:blocks(2))) ,std( mu_pop_avg(:,1:blocks(2)))./sqrt(size(mu_pop_avg,1) )./scale,{'color','g'}); hold on;
figure;
hold on, shadedErrorBar(1:blocks(2), mean(anchors_no_pop(:,1:blocks(2))) ,std( anchors_no_pop(:,1:blocks(2)))./sqrt(size(anchors_no_pop,1) )./scale,{'color','g'}); hold on;
figure;
hold on, shadedErrorBar(1:blocks(2), mean(hd_pop_avg(:,1:blocks(2))) ,std( hd_pop_avg(:,1:blocks(2)))./sqrt(size(hd_pop_avg,1) )./scale,{'color','g'}); hold on
    end
    

end

end

save ("hd_accuracy_anchors_no_variable_2_15_dist_criterion.mat","hd_pop_avg","corr_pop_avg","anchors_no_pop");
scale = 1;  
figure;
% corr_pop_avg=permute(corr_pop_avg,[2 3 1]);
hold on, shadedErrorBar(1:blocks(1),mean(corr_pop_avg(:,1:blocks(1))),std(corr_pop_avg(:,1:blocks(1)))./sqrt(size(corr_pop_avg,1))./scale,{'color','r'}); hold on;
hold on, shadedErrorBar(blocks(1)+1:blocks(2)+blocks(1), mean(corr_pop_avg(:,blocks(1)+1:end)) ,std( corr_pop_avg(:,blocks(1)+1:end))./sqrt(size(corr_pop_avg,1) )./scale,{'color','g'}); hold on;

figure;
hold on, shadedErrorBar(1:blocks(1),mean(hd_pop_avg(:,1:blocks(1))),std(hd_pop_avg(:,1:blocks(1)))./sqrt(size(hd_pop_avg,1))./scale,{'color','r'}); hold on;
hold on, shadedErrorBar(blocks(1)+1:blocks(2)+blocks(1), mean(hd_pop_avg(:,blocks(1)+1:end)) ,std( hd_pop_avg(:,blocks(1)+1:end))./sqrt(size(hd_pop_avg,1) )./scale,{'color','g'}); hold on;

figure;
hold on, shadedErrorBar(1:blocks(1),mean(anchors_no_pop(:,1:blocks(1))),std(anchors_no_pop(:,1:blocks(1)))./sqrt(size(anchors_no_pop,1))./scale,{'color','r'}); hold on;
hold on, shadedErrorBar(blocks(1)+1:blocks(2)+blocks(1), mean(anchors_no_pop(:,blocks(1)+1:end)) ,std( anchors_no_pop(:,blocks(1)+1:end))./sqrt(size(anchors_no_pop,1) )./scale,{'color','g'}); hold on;

figure;
hold on, shadedErrorBar(1:blocks(1),mean(mu_pop_avg(:,1:blocks(1))),std(mu_pop_avg(:,1:blocks(1)))./sqrt(size(mu_pop_avg,1))./scale,{'color','r'}); hold on;
hold on, shadedErrorBar(blocks(1)+1:blocks(2)+blocks(1), mean(mu_pop_avg(:,blocks(1)+1:end)) ,std( mu_pop_avg(:,blocks(1)+1:end))./sqrt(size(mu_pop_avg,1) )./scale,{'color','g'}); hold on;

