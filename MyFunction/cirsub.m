function y=cirsub(x,n,varargin)
if size(varargin)==0
    if x==1
        y=n;
        return
    else
        y=x-1;
    end
else
    i=varargin{1};
    if x<=i
        y=x-i+n;
        return
    else
        y=x-1;
    end
end

