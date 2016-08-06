function odometry = readOdometry(bag, topic)

odometry_raw = bag.readAll(topic);

sz = length(odometry_raw);

odometry.t = zeros(1, sz);
odometry.i = zeros(1, sz);
odometry.p = zeros(3, sz);
odometry.q = zeros(4, sz);
odometry.v = zeros(3, sz);
odometry.w = zeros(3, sz);

for i=1:sz
   odometry.t(i) = timestampFromHeader(odometry_raw{i}.header);
   odometry.i(i) = odometry_raw{i}.header.seq;
   odometry.p(:,i) = odometry_raw{i}.pose.pose.position;
   odometry.q(:,i) = odometry_raw{i}.pose.pose.orientation;
   odometry.v(:,i) = odometry_raw{i}.twist.twist.linear;
   odometry.w(:,i) = odometry_raw{i}.twist.twist.angular;
end