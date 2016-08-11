function quat = unwrap_quat(quat)
%get the sign on the quaternion to be continuous

dq = sum(diff(quat,[],2).^2);
flips = find(dq>1);
sq = zeros(1,size(quat,2)-1);
sq(flips(1:2:end)) = -2;
sq(flips(2:2:end)) = 2;
sq = cumsum([1 sq]);
quat = quat.*sq(ones(size(quat,1),1),:);
