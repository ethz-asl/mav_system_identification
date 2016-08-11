function q = rot2quat(R)
%q = rot2quat(R)

Rv = reshape(R,9,[]);

% the cubed root determinate of R is the scaling factor
detR = sum(Rv([1 4 7],:).*Rv([5 8 2],:).*Rv([9 3 6],:))- ...
       sum(Rv([7 1 4],:).*Rv([5 8 2],:).*Rv([3 6 9],:));
Q2 = detR.^(1/3);


q = sqrt(max(0,[(Q2+Rv(1,:)+Rv(5,:)+Rv(9,:))
                (Q2+Rv(1,:)-Rv(5,:)-Rv(9,:))
                (Q2-Rv(1,:)+Rv(5,:)-Rv(9,:))
                (Q2-Rv(1,:)-Rv(5,:)+Rv(9,:))]))/2;

% now copy signs
g = find(Rv(6,:)<Rv(8,:)); q(2,g) = -q(2,g);
g = find(Rv(7,:)<Rv(3,:)); q(3,g) = -q(3,g);
g = find(Rv(2,:)<Rv(4,:)); q(4,g) = -q(4,g);

Q2 = sum(q.^2);

% normalize
D = 0.5*(1-Q2);
q = q + q.*D([1 1 1 1],:);

return

q0 = sqrt(R(1,1,:)+R(2,2,:)+R(3,3,:)+1)/2;

q1 = (R(3,2,:)-R(2,3,:))/4./q0;
q2 = (R(1,3,:)-R(3,1,:))/4./q0;
q3 = (R(2,1,:)-R(1,2,:))/4./q0;

i = find(q0 == 0);
if ~isempty(i)
   q1(i) = sqrt(abs(.5 *(R(2,2,i)+R(3,3,i))));
   q2(i) = sqrt(abs(.5 *(R(1,1,i)+R(3,3,i))));
   q3(i) = sqrt(abs(.5 *(R(1,1,i)+R(2,2,i))));
   
   j = i(find(q1(i) ~= 0));
   if ~isempty(j)
      q2(j) = q2(j) .* sign(R(1,2,j));
      q3(j) = q3(j) .* sign(R(1,3,j));
   end
   
   j = i(find(q1(i) == 0));
   if ~isempty(j)
      q3(j) = q3(j) .* sign(R(2,3,j));
   end
end
q = [q0(:)'; q1(:)'; q2(:)'; q3(:)'];