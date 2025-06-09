function filtered_table = medianFilter(table, window_size)
    % MEDIANFILTER applies a median filter to the data in the table.
    %
    % Inputs
    % ------
    %   table   - Table of data derived from .csv file (after outlier
    %             removal).
    %   window_size - Odd integer. The size of the moving window.
    %
    % Outputs
    % -------
    %   filtered_table  - Table with median filtered data.
    A = table2array(table);
    W = size(A, 2);

    for i = 1:W
        A(:, i) = medfilt1(A(:, i), window_size);
    end

    filtered_table = array2table(A, "VariableNames", ...
        {'d', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'});