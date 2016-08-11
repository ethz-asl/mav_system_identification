function S = skew(w)
%SKEW   Forms skew symmetric matrix.
% S = SKEW (w);  S is skew symmetric matrix from vector w

if any(size(w) == 1)
  S = [  0 -w(3) w(2);
       w(3)   0 -w(1);
      -w(2) w(1)   0];
else
 S = [w(3,2) w(1,3) w(2,1)]'; 
end

  
return

n = rand(3,1);
t = rand(3,1);
w = rand(3,1);

NN = n*t';
O = skew(w);
P = -O + NN;

Ps = P+P';
[Q,L] = eig(Ps);
L = diag(L)
[L,i] = sort(L); Q = Q(:,i);
