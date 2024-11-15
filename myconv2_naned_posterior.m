function outMat=myconv2_naned_posterior(inMat,sigma,arena_mask,home_mask)

%nan inMat where the masks are ones
inMat(find(arena_mask))=nan;
inMat(find(home_mask))=nan;

%carry out the convolution on this new naned matrix

filtersize = round(3.5*sigma); %

% create gaussian filter kernel
R = floor((ceil((filtersize-1)/2)*2+1)/2);
[xx yy] = meshgrid(-R(2):R(2),-R(1):R(1));
fk = exp(-(xx.^2/(2*sigma(2)^2) + yy.^2/(2*sigma(1)^2)));
fk = fk/sum(fk(:));
filtersize = size(fk); % filter size is forced to be odd
% pad the image so that the filter can run over the edge
padsize = floor(filtersize/2);
paddedimage = padarray(inMat,padsize,'replicate','both');
s0 = size(inMat);
outMat = zeros(s0,'double');
os = filtersize-1; 
paddedimage = double(paddedimage);

for m = 1:s0(1)
    for n = 1:s0(2)
        if (~isnan(inMat(m,n)))
        sample = paddedimage(m:(m+os(1)),n:(n+os(2)));
        outMat(m,n) = nansum(sample.*fk,'all');
        end
    end
end



outMat=outMat./sum(outMat(:));


end