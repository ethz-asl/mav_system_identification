function a = autocorr(x)
a = zeros(size(x));
N = size(x,2);

for i=1:size(x,1);
  m = mean(x(i,:));
  c = conv(x(i,:)-m,fliplr(x(i,:))-m);
  a(i,:) = c(N:end)./(N:-1:1);
end

return 
X = zeros(length(x)-numlags+1,numlags);

for i=1:numlags
  X(:,i) = x(i:end-numlags+i);
end

a = X'*X/length(X);

