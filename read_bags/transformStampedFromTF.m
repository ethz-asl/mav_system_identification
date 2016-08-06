function tf = transformStampedFromTF(bag, topic, frame_id, child_frame_id)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);
counter=1;

for i=1:sz
   if strcmp(tf_raw{i}.transforms.header.frame_id, frame_id) && ...
           strcmp(tf_raw{i}.transforms.child_frame_id,child_frame_id)
       tf.t(counter) = timestampFromHeader(tf_raw{i}.transforms.header);
       tf.i(counter) = counter;
       tf.p(:,counter) = tf_raw{i}.transforms.transform.translation;
       tf.q(:,counter) = tf_raw{i}.transforms.transform.rotation;
       counter = counter + 1;
   end
end