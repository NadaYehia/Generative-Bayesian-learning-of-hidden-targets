function result = nanconv2(A, K)
    % NANCONV2 Performs 2D convolution while ignoring NaNs
    % A - Input matrix (can contain NaNs)
    % K - Kernel/filter for convolution
    % Create a binary mask where NaNs are 0, and valid values are 1
    mask = ~isnan(A);
    % Replace NaNs with zeros for convolution
    A(isnan(A)) = 0;
    % Perform normal convolution
    convA = conv2(A, K, 'same');
    convMask = conv2(mask, K, 'same'); % Normalization factor
    % Normalize to handle missing data
    result = convA ./ convMask;
    % Preserve NaNs where the normalization factor is zero
    result(convMask == 0) = NaN;
end