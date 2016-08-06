function a = xcorr(x,y)
a = zeros(size(x,1),size(x,2)+size(y,2)-1);
N = size(x,2);

for i=1:size(x,1);
  c = conv(x(i,:),fliplr(y(i,:)));
  a(i,:) = c;
end

return 
X = zeros(length(x)-numlags+1,numlags);

for i=1:numlags
  X(:,i) = x(i:end-numlags+i);
end

a = X'*X/length(X);

