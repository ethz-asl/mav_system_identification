function out = quatrot(q,v)

if size(v,1)~=3
	display('v must be size 3xN');
	return
end
res = mul_quat(mul_quat(q,[zeros(1,size(v,2));v]),diag([1 -1 -1 -1])*q);
out = res(2:4,:);



%%%% now this is somewhat wrong...
% % t2 =   q(1,:).*q(2,:);
% % t3 =   q(1,:).*q(3,:);
% % t4 =   q(1,:).*q(4,:);
% % t5 =  -q(2,:).*q(2,:);
% % t6 =   q(2,:).*q(3,:);
% % t7 =   q(2,:).*q(4,:);
% % t8 =  -q(3,:).*q(3,:);
% % t9 =   q(3,:).*q(4,:);
% % t10 = -q(4,:).*q(4,:);
% % v1 = 2*( (t8 + t10).*v(1,:) + (t6 -  t4).*v(2,:) + (t3 + t7).*v(3,:) ) + v(1,:);
% % v2 = 2*( (t4 +  t6).*v(1,:) + (t5 + t10).*v(2,:) + (t9 - t2).*v(3,:) ) + v(2,:);
% % v3 = 2*( (t7 -  t3).*v(1,:) + (t2 +  t9).*v(2,:) + (t5 + t8).*v(3,:) ) + v(3,:);
% % out = [v1;v2;v3];
