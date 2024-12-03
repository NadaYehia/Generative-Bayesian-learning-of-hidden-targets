
function f = makeimagestack(m,wantnorm,addborder,csize,bordersize,thts,rs)
 
 % input
 if ~exist('wantnorm','var') || isempty(wantnorm)
   wantnorm = 0;
 end
 if ~exist('addborder','var') || isempty(addborder)
   addborder = 1;
 end
 if ~exist('csize','var') || isempty(csize)
   csize = -1;
 end
 if ~exist('bordersize','var') || isempty(bordersize)
   bordersize = 0;
 end
 
 % calc
 nrows = size(m,1);
 ncols = size(m,2);
 
 % make double if necessary
 m = double(m);
 wantnorm = double(wantnorm);
 
 % make <m> 3D if necessary
 m = reshape(m,size(m,1),size(m,2),[]);
 
 % find range, normalize
 if length(wantnorm)==2
   m = normalizerange(m,0,1,wantnorm(1),wantnorm(2));
   mn = 0;
   mx = 1;
 elseif wantnorm==0
   mn = nanmin(m(:));
   mx = nanmax(m(:));
 elseif wantnorm==-1
   m = normalizerange(m,0,1,-max(abs(m(:))),max(abs(m(:))));
   mn = 0;
   mx = 1;
 elseif wantnorm==-2
   m = normalizerange(m,0,1,0,max(m(:)));
   mn = 0;
   mx = 1;
 elseif wantnorm==-3
   m = normalizerange(m,0,1,min(m(:)),max(m(:)));
   mn = 0;
   mx = 1;
 else
   rng = prctile(m(:),[wantnorm 100-wantnorm]);
   if rng(2)==rng(1)
     m = zeros(size(m));  % avoid error from normalizerange.m
   else
     m = normalizerange(m,0,1,rng(1),rng(2));
   end
   mn = 0;
   mx = 1;
 end
 md = (mn+mx)/2;
 
 % number of images
 numim = size(m,3);
 
 % calculate csize if necessary
 if isempty(csize)
   rows = floor(sqrt(numim));  % MAKE INTO FUNCTION?
   cols = ceil(numim/rows);
   csize = [rows cols];
 elseif isequal(csize,-1)
   csize = [numim 1];
 elseif csize(1)==0
  csize(1) = ceil(numim/csize(2));
 elseif csize(2)==0
   csize(2) = ceil(numim/csize(1));
 end
 
 % calc
 chunksize = prod(csize);
 numchunks = ceil(numim/chunksize);
 
 % convert to cell vector, add some extra matrices if necessary
 m = splitmatrix(m,3);
 m = [m repmat({repmat(mn,size(m{1}))},1,numchunks*chunksize-numim)];
 
 % figure case
 if isnan(addborder)
 
   for p=1:numchunks
     if p ~= 1
       drawnow; figure;
     end
     hold on;
     for q=1:chunksize
       xx = linspace(1+(ceil(q/csize(1))-1)*(ncols+1),ncols+(ceil(q/csize(1))-1)*(ncols+1),ncols);
       yy = linspace(1+(mod2(q,csize(1))-1)*(nrows+1),nrows+(mod2(q,csize(1))-1)*(nrows+1),nrows);
       
       imagesc(xx,yy,m{(p-1)*chunksize+q},[0 1]);
     end
     axis equal;
     set(gca,'YDir','reverse');
   end
   f = [];
 
 % matrix case
 else
 
   % add border?
   if imag(addborder) || addborder
     for p=1:length(m)
       m{p}(end+(1:bordersize),:) = choose(imag(addborder),0,choose(addborder > 0,mx,md));
       m{p}(:,end+(1:bordersize)) = choose(imag(addborder),0,choose(addborder > 0,mx,md));
     end
   end
   
   % combine images
   f = [];
   for p=1:numchunks
     temp = m((p-1)*chunksize + (1:chunksize));
     f = cat(3,f,cell2mat(reshape(temp,csize)));
   end
   
   % remove final?
   if abs(addborder)==2
     f(end-bordersize+1:end,:,:) = [];
         f(:,end-bordersize+1:end,:) = [];
       end 
 end

 function f = splitmatrix(m,dim,splt)

 % <m> is a matrix
 % <dim> is a dimension
 % <splt> (optional) is a vector of positive integers indicating
 %   how to perform the split.  default: ones(1,size(m,dim)).
 %   you can also flip the sign of entries to indicate that you
 %   do not want that entry returned.  special case is <splt>==0
 %   which means use <splt> equal to size(m,dim).
 %
 % split <m> along dimension <dim>, returning a cell vector of matrices.
 %
 % example:
 % isequal(splitmatrix([1 2; 3 4],2),{[1 3]' [2 4]'})
 % isequal(splitmatrix([1 2 3 4],2,[2 -1 1]),{[1 2] [4]})
 
 % input
 if ~exist('splt','var') || isempty(splt)
   splt = [];  % deal with later
 end
 if isequal(splt,0)
   splt = size(m,dim);
 end
 
 % what is the max number of dimensions involved?
 maxdim = max(ndims(m),dim);                % 5
 
 % figure out the dimensions of m
 msize = ones(1,maxdim);
 msize(1:ndims(m)) = size(m);               % [50 60 40 1 2]
 
 % convert to cell
 msize = num2cell(msize);                   % {50 60 40 1 2}
 
 % hack it in
 if isempty(splt)
   splt = ones(1,size(m,dim));
 end
 msize{dim} = abs(splt);                    % {50 60 40 1 [1 1]}
 
 % do it
   prev = warning('query'); warning('off');
 f = flatten(mat2cell(m,msize{:}));
   warning(prev);
f = f(splt > 0);


function f = flatten(m)

f = m(:).';

function f = choose(flag,yes,no)

 if flag
   f = yes;
 else
   f = no;
 end


