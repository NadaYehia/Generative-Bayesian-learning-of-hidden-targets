% Given a stream of integers and a window size, 
% calculate the moving average of all integers in the sliding window.
%  
% x = [1, 3, 5, 10, 2, 11]
% movingAvg(x, n)
% output: [3.0, 6.0, 5.67, 7.67] , for n=3

function output=movingAvg (x,n)
  m=(numel(x)-n)+1;
  output=zeros(1,m);

    for j=1:m
        output(1,j)=sum(x(j:j+n-1))/n;

    end

end


