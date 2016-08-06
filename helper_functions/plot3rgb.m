function plot3rgb(M,rgb,varargin)

dims = size(M);

x = M(1,:)';
y = M(2,:)';
z = M(3,:)';

use_vtk = 0;
if (exist('vtk_plot3')), use_vtk = 1; end

[c,cmap] = rgb2ind(rgb,256);

S = sparse(1:length(c),c,c);

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
