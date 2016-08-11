function qn = renorm(q)
% qn = renorm(q)
% 		This renormalizes each quaternion such that it has unit magnitude;

mag = sqrt(sum(q.^2));
qn = reshape(q(:,:) ./ mag(ones(size(q,1),1),:), size(q));

