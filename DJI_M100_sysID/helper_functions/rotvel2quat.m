function q = rotvel2quat(rot)

halfangle = 0.5*sqrt(sum(rot.^2));
halfangle(find(~halfangle))=2*pi;

q = [cos(halfangle); rot .* repmat(sin(halfangle)./(2*halfangle),[3 1])];
