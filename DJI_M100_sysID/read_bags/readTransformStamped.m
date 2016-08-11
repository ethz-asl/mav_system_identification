function tf = readTransformStamped(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

tf.t = zeros(1, sz);
tf.i = zeros(1, sz);
tf.p = zeros(3, sz);
tf.q = zeros(4, sz);

for i=1:sz
   tf.t(i) = timestampFromHeader(tf_raw{i}.header);
   tf.i(i) = tf_raw{i}.header.seq;
   tf.p(:,i) = tf_raw{i}.transform.translation;
   tf.q(:,i) = tf_raw{i}.transform.rotation;
end