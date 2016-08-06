% takes a vector of length 3 and converts to 4x4 skew symetric matrix


function qskew = quatskew(q)
%convention: first element is real part:
qskew=zeros(4);
qskew(1,2:4)=-q(2:4);
qskew(2,3)=q(4);
qskew(2,4)=-q(3);
qskew(3,4)=q(2);
qskew=qskew-qskew';
qskew=qskew+diag(ones(1,4)*q(1));



% convention: last element is real part:
% vec=q;
% qskew=zeros(4);
% qskew(1,2)=vec(3);
% qskew(1,3)=-vec(2);
% qskew(1,4)=vec(1);
% qskew(2,3:4)=vec(1:2);
% qskew(3,4)=vec(3);
% qskew=qskew-qskew';
