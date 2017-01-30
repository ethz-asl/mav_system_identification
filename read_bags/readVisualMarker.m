function vc_pose = readVisualMarker(bag, topic)

vm_raw = bag.readAll(topic);

sz = length(vm_raw);

vc_pose.t = zeros(1, sz);
vc_pose.i = zeros(1, sz);
vc_pose.p = zeros(3, sz);
vc_pose.q = zeros(4, sz);

for i=1:sz
   vc_pose.t(i) = timestampFromHeader(vm_raw{i}.header);
   vc_pose.i(i) = vm_raw{i}.header.seq;
   vc_pose.p(:,i) = vm_raw{i}.pose.position;
   vc_pose.q(:,i) = vm_raw{i}.pose.orientation;
end