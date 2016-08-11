function tf = readDoubleArrayStamped(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

tf.t = zeros(1, sz);
tf.data = zeros(31, sz);


for i=1:sz
   tf.t(i) = timestampFromHeader(tf_raw{i}.header);
   tf.data(:,i) = tf_raw{i}.data;
end