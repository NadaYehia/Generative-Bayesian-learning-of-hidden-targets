function filt_locs=filter_peaks(row_y,col_x,data,threshold)
    % set the nan values to 0 to allow values comparsions
    data(isnan(data))=0;
    
    locs=sub2ind(size(data),row_y,col_x);
    delta_y=[-1 -1 -1 0 0 +1 +1 +1];
    delta_x=[-1 0 +1 -1 +1 -1 0 +1];
    % add these deltas to the detected peak positions to get the
    % coordinates of their immediate neighbors:
    neighbor_locs_y=row_y+delta_y;
    neighbor_locs_x=col_x+delta_x; clear row_y col_x delta_x delta_y
    % ...check for indices beyond array
    % borders:
    neighbor_locs_y(neighbor_locs_y<1)=1;
    neighbor_locs_y(neighbor_locs_y>size(data,1))=size(data,1);
    neighbor_locs_x(neighbor_locs_x<1)=1;
    neighbor_locs_x(neighbor_locs_x>size(data,2))=size(data,2);
    % ...convert to linear indices:
    neighbor_locs=sub2ind(size(data),neighbor_locs_y,neighbor_locs_x);
    clear neighbor_locs_y neighbor_locs_x
    
    % keep the peaks which are greater than their neighbors by the threshold
    % parameter
    suitable=(data(locs)-threshold>=data(neighbor_locs));
    % Now check for those cases when by mistake (earlier step for checking
    % for indices beyond array boundaries) a peak itself is taken as a
    % neighbor as well:
    suitable(data(neighbor_locs)==data(locs))=true;
    
    % Only keep those is they are greater than ALL neighbors (in other
    % words, those elements where NONE are lesser):
    suitable=~any(~suitable,2); clear neighbor_locs
    
    % Final step - locate suitable element indices:
    suitable=find(suitable);
    
    % That's it, modify the output array:
    filt_locs=locs(suitable); clear suitable