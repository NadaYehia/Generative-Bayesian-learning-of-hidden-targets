% inputs
function outMat= myconv2 (inMat,sigma)

filtersize = round(4*sigma); %

% create gaussian filter kernel
R = floor((ceil((filtersize-1)/2)*2+1)/2);
[xx yy] = meshgrid(-R(2):R(2),-R(1):R(1));
fk = exp(-(xx.^2/(2*sigma(2)^2) + yy.^2/(2*sigma(1)^2)));
fk = fk/sum(fk(:));
filtersize = size(fk); % filter size is forced to be odd
% pad the image so that the filter can run over the edge
padsize = floor(filtersize/2);
paddedimage = padarray(inMat,padsize,0,'both');
s0 = size(inMat);
outMat = zeros(s0,'double');
os = filtersize-1; 
paddedimage = double(paddedimage);
for m = 1:s0(1)
    for n = 1:s0(2)
        sample = paddedimage(m:(m+os(1)),n:(n+os(2)));
        outMat(m,n) = sum(sample.*fk,'all');
    end
end



outMat=outMat./nansum(outMat(:));
end