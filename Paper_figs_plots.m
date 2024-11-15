%%
save("peak_anchors_illustr_i_new_a.mat","top_anchrs","rnd_anchrs","mus","omegas");

%%
save("new_path_anchors_a_new.mat","r","theta");

%%
save("new_path_full_traj_likelihood_a_new.mat","r","omega");


%% plot new trajectory planned for trial i+1, with anchors overlaid on x,y 
targets=env.setup_targets_coord;
target_num=1;
arena_size=env.arena_dimensions;

arenacoord.x=[arena_size(1) arena_size(2) arena_size(2) arena_size(1)];
arenacoord.y=[arena_size(3) arena_size(3) arena_size(4) arena_size(4)];
f2=figure;
f2.Position=[100 100 300 300];

hold on, patch(arenacoord.x,arenacoord.y,[1 1 1]); hold on;
hold on, patch(targets(target_num).x,targets(target_num).y,[0.85 0.9 0.9]);
plot(x_op,y_op,'k','LineWidth',1.5);
hold on, scatter(x_,y_,'r','filled');
set(gca,'XTick',[]);
set(gca,'YTick',[]);



%% plot heading and speed controls for trajectory i+1
f3=figure;
f3.Position=[100 100 300 300];
subplot(2,1,1),plot(speed_conca,'k','LineWidth',1.5); xlim([1 numel(speed_conca)])
set(gca,'TickDir','out')
fontsize(gca,18,'points');
fontname(gca,'Times New Roman');
ylabel(gca,'speed')
xlabel(gca,'time')
box off;
subplot(2,1,2),plot(heading_conca,'k','LineWidth',1.5);xlim([1 numel(speed_conca)])
set(gca,'TickDir','out')
fontsize(gca,18,'points');
fontname(gca,'Times New Roman');
yticks([-pi  0  pi])
yticklabels({'-\pi','0','\pi'})
ylabel(gca,'heading angle')
xlabel(gca,'time')
ylim([-pi pi])
box off;
%% plot current belief without anchors

nan_mask2=double(my_mask2);
nan_mask2((my_mask2))=nan;

nan_mask=double(my_mask);
nan_mask((my_mask))=nan;

naned_prior=prior+nan_mask2;
naned_prior=naned_prior+nan_mask;
naned_prior=naned_prior(:,1:80); %from r=1 to 800

f4=figure(3);
% cb(:,:,1)=flipud(naned_prior);
% imagesc3D(cb,fliplr(Os),Rs(1:80))
f4.Position=[100 100 300 300];
f4.InnerPosition=[30 30 200 200];
h=imagesc(fliplr(Os),Rs(1:80),fliplr(naned_prior'));
set(h, 'AlphaData', ~isnan(naned_prior'));
set(gca,'YDir','normal');
set(gca,'XDir','reverse');
% xticks([-pi/2 -pi/4  0  pi/4  pi/2])
% xticklabels({'-\pi/2','-\pi/4','0','\pi/4','\pi/2'})
% set(gca,'TickDir','out')
% set(gca,'TickDir','out')
% fontsize(gca,18,'points');
% fontname(gca,'Times New Roman');
xlabel('θ');
ylabel('R');
set(gca,'XTick',[]);
set(gca,'YTick',[]);
% colorbar;
box off;
%%
load("peak_anchors_illustr_i_new_a.mat");
hold on;
scatter(Os(omegas( top_anchrs)),Rs(mus(top_anchrs)),30,'MarkerEdgeColor','k');
hold on, scatter(Os(omegas( top_anchrs(rnd_anchrs))),Rs(mus(top_anchrs(rnd_anchrs))),30,'MarkerFaceAlpha',0.5,'MarkerFaceColor',[0.5 0.5 0.5]);
%%

f5=figure(4);
f5.Position=[100 100 300 300];
f5.InnerPosition=[30 30 200 200];
h=imagesc(fliplr(Os),Rs(1:80),fliplr(naned_prior'));
set(h, 'AlphaData', ~isnan(naned_prior'));
set(gca,'YDir','normal');
set(gca,'XDir','reverse');
set(gca,'YTick',[]);
set(gca,'XTick',[]);
hold on, scatter(omega_anchors,r_anchors,30,'r','filled');
box off;

%%
save("curr_path_likelihood.mat","L1");
%%
save("new_path_likelihood2.mat","L1");
%% plot current likelihood 
load("curr_path_likelihood.mat","L1");
nan_mask2=double(my_mask2);
nan_mask2((my_mask2))=nan;

nan_mask=double(my_mask);
nan_mask((my_mask))=nan;

naned_L1=(1-L1)+nan_mask2;
naned_L1=naned_L1+nan_mask;
naned_L1=naned_L1(:,1:80);

f6=figure;
f6.Position=[100 100 300 300];
f6.InnerPosition=[30 30 200 200];
h=imagesc(fliplr(Os),Rs(1:80),fliplr((naned_L1)'));
set(h, 'AlphaData', ~isnan((naned_L1)'));
set(gca,'YDir','normal');
set(gca,'XDir','reverse');
set(gca,'YTick',[]);
set(gca,'XTick',[]);
% colorbar;
%%
hold on, scatter(omega_anchors,r_anchors,30,'r','filled');
hold on, plot(omegas_,rs_,'--k','LineWidth',1.5);
%%
load("new_path_anchors_a_new.mat","r","theta");
hold on, scatter(theta-(pi/2),r,30,'r','filled');
load("new_path_full_traj_likelihood_a_new.mat","r","omega");
hold on, plot(omega,r,'--k','LineWidth',1.5);



%% plot next belief as 3D slice
f7=figure(5);
f7.Position=[100 100 300 300];
f7.InnerPosition=[30 30 200 200];
nan_mask2=double(my_mask2);
nan_mask2((my_mask2))=nan;

nan_mask=double(my_mask);
nan_mask((my_mask))=nan;

naned_prior=prior+nan_mask2;
naned_prior=naned_prior+nan_mask;
naned_prior=naned_prior(:,1:80);

h=imagesc(fliplr(Os),Rs(1:80),fliplr(naned_prior'));
set(h, 'AlphaData', ~isnan(naned_prior'));
set(gca,'YDir','normal');
set(gca,'XDir','reverse');
set(gca,'YTick',[]);
set(gca,'XTick',[]);
box off;












