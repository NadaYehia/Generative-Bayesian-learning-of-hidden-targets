function filt_locs=filter_peaks_with_distance(row_y,col_x,data,min_distance)
    
% filter_peaks_with_distance - Filters peaks based on a minimum distance criterion.
%                              Peaks that are too close to each other are removed, keeping only the highest one.
%
% Inputs:
%   row_y         - Row indices of the detected peaks.
%   col_x         - Column indices of the detected peaks.
%   data          - 2D matrix (e.g., posterior distribution) from which peaks were detected.
%   min_distance  - Minimum allowed distance between peaks (in pixels).
%
% Output:
%   filt_locs     - Linear indices of the filtered peaks.



%%
    % Handle NaN values in the data by setting them to 0.
    data(isnan(data))=0;

    % Convert the row and column indices of peaks to linear indices.
    locs=sub2ind(size(data),row_y,col_x);

    % Extract the peak values from the data using the linear indices
    pks=data(locs);

    % Sort the peaks in descending order of their values.
    [pks_sorted,idx]=sort(pks,'descend');
    
    % Sort the linear indices of the peaks according to the sorted peak values.
    locs_sorted=locs(idx); clear idx

    % Convert the sorted linear indices back to row and column indices.
    [row_y,col_x]=ind2sub(size(data),locs_sorted);

    % Start from the highest peak:
    this_peak=1;
        
        % Loop through all peaks.
        while this_peak<(length(pks_sorted)+1)

        % Calculate the Cartesian distance between the current peak and all other peaks.

        dist=sqrt((row_y-row_y(this_peak)).^2+(col_x-col_x(this_peak)).^2);

        % Identify peaks that are within the minimum distance threshold and 
        % are not the current peak itself.

        within=( (dist<=min_distance) & (dist~=0) );

        % Remove peaks that are too close to the current peak.
        pks_sorted(within)=[];
        locs_sorted(within)=[];
        row_y(within)=[]; col_x(within)=[];

        % Move to the next peak in the sorted list.
        this_peak=this_peak+1;

        end

        % Clear temporary variables.
        clear this_peak within dist row_y col_x
    
       % Return the linear indices of the filtered peaks.
       filt_locs=locs_sorted;
end
