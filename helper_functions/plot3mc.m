function hs = plot3mc(M,varargin)

dims = size(M);

x = M(1,:)';
y = M(2,:)';
z = M(3,:)';

if (dims(1) > 3)
  c = M(4,:)';
else
  c = -z;
end

use_vtk = 0;
if (exist('vtk_plot3')), use_vtk = 1; end
   
c = floor((c-min(c))/(max(c)-min(c))*255)+1;
S = sparse(1:length(c),c,c);
cmap = jet(256);
%cmap = hsv(256);

held = ishold;

h = [];
for i=1:256;
  g = find(S(:,i));
  if ~isempty(g)
    
    if use_vtk
      vtk_plot3(x(g),y(g),z(g),'.',varargin{:},'MarkerColor',cmap(i,:));
    else
      hi = plot3(x(g),y(g),z(g),varargin{:},'color',cmap(i,:));
      h = [h hi];
    end
    hold on;
  end
end

if ~held
  hold off
end


if nargout
    hs = h;
end
