function h = plot3lc(P1,P2,C,varargin)

mnC = min(C(:)); mxC = max(C(:));
C = round((C-mnC)*255/(mxC-mnC)) + 1;

cmap = jet(256);

held = ishold;

for i=1:256
  g = find(C==i);
  if isempty(g), continue; end

  plot3m(reshape(permute(cat(3,P1(1:3,g),P2(1:3,g),nan*zeros(3,length(g))),[1 3 2]),3,[]),varargin{:},'color', cmap(i,:));
  hold on;
end

if ~held, hold off; end

