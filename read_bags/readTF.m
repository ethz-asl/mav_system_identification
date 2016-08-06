function tf = readTF(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

tf.t = zeros(1, sz);
tf.i = zeros(1, sz);
tf.frame_id = cell(1, sz);
tf.child_frame_id = cell(1, sz);

tf.p = zeros(3, sz);
tf.q = zeros(4, sz);

for i=1:sz
   tf.t(i) = timestampFromHeader(tf_raw{i}.transforms.header);
   tf.i(i) = tf_raw{i}.transforms.header.seq;
   tf.frame_id{i} = tf_raw{i}.transforms.header.frame_id;
   tf.child_frame_id{i} = tf_raw{i}.transforms.child_frame_id;
   tf.p(:,i) = tf_raw{i}.transforms.transform.translation;
   tf.q(:,i) = tf_raw{i}.transforms.transform.rotation;
end