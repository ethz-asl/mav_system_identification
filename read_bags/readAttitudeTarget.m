function attitude_target = readAttitudeTarget(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

attitude_target.t = zeros(1, sz);
attitude_target.i = zeros(1, sz);
attitude_target.q = zeros(4, sz);
attitude_target.thrust = zeros(1, sz);

for i=1:sz
   attitude_target.t(i) = timestampFromHeader(tf_raw{i}.header);
   attitude_target.i(i) = tf_raw{i}.header.seq;
   attitude_target.q(:,i) = tf_raw{i}.orientation;
   attitude_target.thrust(i) = tf_raw{i}.thrust;
end