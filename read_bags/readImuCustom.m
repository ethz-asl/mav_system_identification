function imu = readImuCustom(bag, topic)

imu_raw = bag.readAll(topic);

sz = length(imu_raw);

imu.t = zeros(1, sz);
imu.i = zeros(1, sz);
imu.a = zeros(3, sz);
imu.w = zeros(3, sz);
imu.q = zeros(4, sz);
imu.height = zeros(1, sz);
imu.differential_height = zeros(1, sz);

for i=1:sz
   imu.t(i) = timestampFromHeader(imu_raw{i}.header);
   imu.i(i) = imu_raw{i}.header.seq;
   imu.a(:,i) = imu_raw{i}.acceleration;
   imu.w(:,i) = imu_raw{i}.angular_velocity;
   imu.q(:,i) = imu_raw{i}.orientation;
   imu.height(i) = imu_raw{i}.height;
   imu.differential_height(i) = imu_raw{i}.differential_height;
end