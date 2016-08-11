function point = readPointStamped(bag, topic)

tf_raw = bag.readAll(topic);

sz = length(tf_raw);

point.t = zeros(1, sz);
point.i = zeros(1, sz);
point.p = zeros(3, sz);

for i=1:sz
   point.t(i) = timestampFromHeader(tf_raw{i}.header);
   point.i(i) = tf_raw{i}.header.seq;
   point.p(:,i) = tf_raw{i}.point;
end