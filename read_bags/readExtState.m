function ekf = readExtState(bag, topic)

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



for i=1:sz
   ekf.t(i) = timestampFromHeader(ekf_raw{i}.header);
   ekf.i(i) = ekf_raw{i}.header.seq;
   ekf.state.p(:,i) = ekf_raw{i}.pose.position(1:3);
   ekf.state.v(:,i) = ekf_raw{i}.velocity(1:3);
   ekf.state.q(:,i) = ekf_raw{i}.pose.orientation(1:4);
end