function plot_ancs_in_x_y_space (mus,hds,tol,arena,targets,target_num,m,n)
figure;
epsi=(tol-pi/2);
hold on
patch(arena.x,arena.y,[0.95 0.9 0.95]); hold on;
patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);

for age=m:n

theta= (hds(age,100:end)) +(pi/2);
    
E_d= ((mus(age,100:end)).*(pi^2).*sinc(epsi/pi))./((4*pi)+(4*(epsi)));

x_(age,:)=E_d.*cos(theta);

y_(age,:)=E_d.*sin(theta);

end

hold on, scatter(x_',y_','filled','MarkerFaceAlpha',0.5);

hold off;


end