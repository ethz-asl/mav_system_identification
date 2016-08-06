function rpy = quat2rpy(qs)
% convert quaternion to roll pitch yaw angles

rpy = [
    atan2(2*(qs(1,:).*qs(2,:)+qs(3,:).*qs(4,:)),1-2*(qs(2,:).^2+qs(3,:).^2));
    asin(2*(qs(1,:).*qs(3,:) - qs(4,:).*qs(2,:)));
    atan2(2*(qs(1,:).*qs(4,:)+qs(2,:).*qs(3,:)),1-2*(qs(3,:).^2+qs(4,:).^2));
];