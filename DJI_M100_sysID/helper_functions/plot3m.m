function hs = plot3m(M, varargin)
% PLOT3M   works like plot3, but has all coordinates in one matrix

dims = size(M);

if (length(dims)==1)
   h = plot3(M(1,:),M(2,:),M(3,:), varargin{:});
else
   h = plot3(squeeze(M(1,:,:)),squeeze(M(2,:,:)),squeeze(M(3,:,:)), varargin{:});
end

if nargout
    hs = h;
end
