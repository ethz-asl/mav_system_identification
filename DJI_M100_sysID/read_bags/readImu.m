function imu = readImu(bag, topic)

imu_raw = bag.readAll(topic);

sz = length(imu_raw);

imu.t = zeros(1, sz);
imu.i = zeros(1, sz);
imu.a = zeros(3, sz);
imu.w = zeros(3, sz);
imu.q = zeros(4, sz);

for i=1:sz
   imu.t(i) = timestampFromHeader(imu_raw{i}.header);
   imu.i(i) = imu_raw{i}.header.seq;
   imu.a(:,i) = imu_raw{i}.linear_acceleration;
   imu.w(:,i) = imu_raw{i}.angular_velocity;
   imu.q(:,i) = imu_raw{i}.orientation;
end