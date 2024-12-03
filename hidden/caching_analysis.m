function caching_analysis(runs,caching_no,r,caching_times,theta,acc,anchors_no,blk_swtch)


    filtered_agents= find(sum(caching_times( :,runs(1):runs(2) ),2)==caching_no);
    
    corr_pop_avg=acc(filtered_agents,runs(1):runs(2));
    r_pop_avg=r(filtered_agents,runs(1):runs(2));
    hd_pop_avg=theta(filtered_agents,runs(1):runs(2));
    anchors_no_pop=anchors_no(filtered_agents,runs(1):runs(2));

    figure;
    plot_population_speed_heading_reward_rates;
    figure;
    plot_shaded_error_bar_mean_std(caching_times ( filtered_agents,runs(1):runs(2) ),'k');
    
    for i=1:numel(blk_swtch)
        hold on, plot(blk_swtch(i).*ones(1,11),[0:0.1:1],'--r','LineWidth',1)
    end

end