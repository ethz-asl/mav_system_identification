function h = plot3l(P1,P2,varargin)


h = plot3m(reshape(permute(cat(3,P1(1:3,:),P2(1:3,:),nan*zeros(size(P1(1:3,:)))),[1 3 2]),3,[]),varargin{:});

