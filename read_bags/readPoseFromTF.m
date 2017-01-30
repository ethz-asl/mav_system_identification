function tf = readPoseFromTF(bag, topic,frame_id,child_frame_id)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

%tf.t = zeros(1, sz);
%tf.i = zeros(1, sz);
%tf.frame_id = cell(1, sz);
%tf.child_frame_id = cell(1, sz);

%tf.p = zeros(3, sz);
%tf.q = zeros(4, sz);
cnt=1;
for i=1:sz
	if strcmp(tf_raw{i}.transforms.header.frame_id,frame_id) && strcmp(tf_raw{i}.transforms.child_frame_id,child_frame_id)
		tf.t(cnt) = timestampFromHeader(tf_raw{i}.transforms.header);
		tf.i(cnt) = tf_raw{i}.transforms.header.seq;
		tf.frame_id{cnt} = tf_raw{i}.transforms.header.frame_id;
		tf.child_frame_id{cnt} = tf_raw{i}.transforms.child_frame_id;
		tf.p(:,cnt) = tf_raw{i}.transforms.transform.translation;
		tf.q(:,cnt) = tf_raw{i}.transforms.transform.rotation;
		cnt=cnt+1;
	end
end