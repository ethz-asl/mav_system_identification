function ctrl = readControl(bag, topic)

ctrl_raw = bag.readAll(topic);

sz = length(ctrl_raw);

ctrl.t = zeros(1, sz);
ctrl.i = zeros(1, sz);
ctrl.p = zeros(3, sz);
ctrl.v = zeros(3, sz);
ctrl.a = zeros(3, sz);
ctrl.j = zeros(3, sz);
ctrl.yaw = zeros(1, sz);
ctrl.yaw_dot = zeros(1, sz);

for i=1:sz
   ctrl.t(i) = timestampFromHeader(ctrl_raw{i}.header);
   ctrl.i(i) = ctrl_raw{i}.header.seq;
   ctrl.p(:,i) = [ctrl_raw{i}.x(1) ctrl_raw{i}.y(1) ctrl_raw{i}.z(1)]';
   ctrl.v(:,i) = [ctrl_raw{i}.x(2) ctrl_raw{i}.y(2) ctrl_raw{i}.z(2)]';
   ctrl.a(:,i) = [ctrl_raw{i}.x(3) ctrl_raw{i}.y(3) ctrl_raw{i}.z(3)]';
   ctrl.j(:,i) = [ctrl_raw{i}.x(4) ctrl_raw{i}.y(4) ctrl_raw{i}.z(4)]';
   ctrl.w(i) = ctrl_raw{i}.yaw(1);
   ctrl.q(i) = ctrl_raw{i}.yaw(2);
end