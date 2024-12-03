clear m_anc o_anc x_ y_
figure(1);
tol=clearnce;
figure(2);
hold on
patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);
hold off

epsi=(tol-pi/2);

for itr=1:200
[mu,om,x,y]=connect_actions_with_smooth_trajectory([0 As(150) 0],[Os(250) Os(250) Os(250)],sigma_ridge,...
speed_step,env,clearnce,Drift,T,Os,As,bestFitDataOffset,bestFitMeanToScaleRatio,bestNormalFitAngleScale);

midT=numel(mu)/2;

m_anc(itr)=mu(midT);
o_anc(itr)=om(midT);

theta= (o_anc(itr)) +(pi/2);
    
E_d= ((m_anc(itr))*(pi^2)*sinc(epsi/pi))/((4*pi)+(4*(epsi)));

x_(itr)=E_d*cos(theta);

y_(itr)=E_d*sin(theta);


end
figure(1);
hold on, scatter(o_anc,m_anc,'b');
hold off;

figure(2)
hold on, scatter(x_,y_,'b','filled');
hold off;

[N,bins]=histcounts(m_anc,[0:5:500],'Normalization','probability');
mean_bins=movmean(bins,2);
figure(3),hold on, plot(mean_bins(2:end),N,'b');
 




    

