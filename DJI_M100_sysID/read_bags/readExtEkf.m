function ekf = readExtEkf(bag, topic)

ekf_raw = bag.readAll(topic);

sz = length(ekf_raw);

ekf.t = zeros(1, sz);
ekf.i = zeros(1, sz);

ekf.w = zeros(3, sz);
ekf.a = zeros(3, sz);
ekf.flag = zeros(1, sz);

ekf.state.p = zeros(3, sz);
ekf.state.v = zeros(3, sz);
ekf.state.q = zeros(4, sz);
ekf.state.b_w = zeros(3, sz);
ekf.state.b_a = zeros(3, sz);


for i=1:sz
   ekf.t(i) = timestampFromHeader(ekf_raw{i}.header);
   ekf.i(i) = ekf_raw{i}.header.seq;
   ekf.a(:,i) = ekf_raw{i}.linear_acceleration;
   ekf.w(:,i) = ekf_raw{i}.angular_velocity;
   ekf.state.p(:,i) = ekf_raw{i}.state(1:3);
   ekf.state.v(:,i) = ekf_raw{i}.state(4:6);
   ekf.state.q(:,i) = ekf_raw{i}.state(7:10);
   ekf.state.b_w(:,i) = ekf_raw{i}.state(11:13);
   ekf.state.b_a(:,i) = ekf_raw{i}.state(14:16);
   ekf.flag(i) = ekf_raw{i}.flag;
end