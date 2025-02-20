function filt_locs=filter_peaks_with_distance(row_y,col_x,data,min_distance)
    data(isnan(data))=0;
    locs=sub2ind(size(data),row_y,col_x);
    pks=data(locs);
    [pks_sorted,idx]=sort(pks,'descend');
    
    locs_sorted=locs(idx); clear idx
    [row_y,col_x]=ind2sub(size(data),locs_sorted);

    % Start from the highest peak:
    this_peak=1;
        
        while this_peak<(length(pks_sorted)+1)

        % Cartesian distances to ALL its remaining & yet unchecked neighbors
        % (including itself):
        dist=sqrt((row_y-row_y(this_peak)).^2+(col_x-col_x(this_peak)).^2);
        % Now simply check which neighbors are WITHIN the MinPeakDistance
        % BUT also nonzero (the peak should not be compared to itself):
        within=( (dist<=min_distance) & (dist~=0) );
        % ...and delete those entries satisfying the condition:
        pks_sorted(within)=[];
        locs_sorted(within)=[];
        row_y(within)=[]; col_x(within)=[];

        % Update the peak counter:
        this_peak=this_peak+1;

        end; clear this_peak within dist row_y col_x

    filt_locs=locs_sorted;
end
