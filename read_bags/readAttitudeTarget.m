function tf = readAttitudeTarget(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

tf.t = zeros(1, sz);
tf.i = zeros(1, sz);
tf.q = zeros(4, sz);
tf.thrust = zeros(1, sz);

for i=1:sz
   tf.t(i) = timestampFromHeader(tf_raw{i}.header);
   tf.i(i) = tf_raw{i}.header.seq;
   tf.q(:,i) = tf_raw{i}.orientation;
   tf.thrust(i) = tf_raw{i}.thrust;
end