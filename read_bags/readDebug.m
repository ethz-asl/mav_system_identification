function dbg = readDebug(bag, topic)

dbg_raw = bag.readAll(topic);

sz = length(dbg_raw);

dbg.t = zeros(1, sz);
dbg.i = zeros(1, sz);

dbg.ref.p = zeros(3, sz);
dbg.ref.v = zeros(3, sz);
dbg.ref.a = zeros(3, sz);
dbg.ref.yaw = zeros(1, sz);
dbg.ref.yaw_dot = zeros(1, sz);

dbg.state.p = zeros(3, sz);
dbg.state.v = zeros(3, sz);
dbg.state.a = zeros(3, sz);
dbg.state.yaw = zeros(1, sz);

dbg.pseudo_cmd.a = zeros(3, sz);
dbg.pseudo_cmd.j = zeros(3, sz);

dbg.motor_cmd = zeros(8, sz);

dbg.outer_cmd.w = zeros(3, sz);
dbg.outer_cmd.T = zeros(1, sz);

dbg.inner_cmd.w_dot = zeros(3, sz);
dbg.inner_cmd.T = zeros(1, sz);


for i=1:sz
   dbg.t(i) = timestampFromHeader(dbg_raw{i}.header);
   dbg.i(i) = dbg_raw{i}.header.seq;
   dbg.motor_cmd(:,i) = dbg_raw{i}.output;
   dbg.state.p(:,i) = dbg_raw{i}.state(1:3);
   dbg.state.v(:,i) = dbg_raw{i}.state(4:6);
   dbg.state.a(:,i) = dbg_raw{i}.state(7:9);
   dbg.state.yaw(i) = dbg_raw{i}.state(10);
   if(length(dbg_raw{i}.pseudo_commands)==4) % ugly workaroud for versions before https://github.com/ethz-asl/asctec_mav_framework_devel/commit/95a42380e2a16c788ea1f25f54a13a0f085cde9d
       dbg.pseudo_cmd.a(:,i) = [dbg_raw{i}.pseudo_commands(1) dbg_raw{i}.pseudo_commands(3) 0]';
       dbg.pseudo_cmd.j(:,i) = [dbg_raw{i}.pseudo_commands(2) dbg_raw{i}.pseudo_commands(4) 0]';
   else
       dbg.pseudo_cmd.a(:,i) = [dbg_raw{i}.pseudo_commands(1) dbg_raw{i}.pseudo_commands(3) dbg_raw{i}.pseudo_commands(5)]';
       dbg.pseudo_cmd.j(:,i) = [dbg_raw{i}.pseudo_commands(2) dbg_raw{i}.pseudo_commands(4) dbg_raw{i}.pseudo_commands(6)]';
   end
   dbg.outer_cmd.w(:,i) = dbg_raw{i}.outer_commands(1:3);
   dbg.outer_cmd.T(i) = dbg_raw{i}.outer_commands(4);
   
   dbg.outer_cmd.w_dot(:,i) = dbg_raw{i}.inner_commands(1:3);
   dbg.outer_cmd.T(i) = dbg_raw{i}.inner_commands(4);

   dbg.ref.p(:,i) = dbg_raw{i}.reference(1:3);
   dbg.ref.v(:,i) = dbg_raw{i}.reference(4:6);
   dbg.ref.a(:,i) = dbg_raw{i}.reference(7:9);
   dbg.ref.yaw(i) = dbg_raw{i}.reference(10);
   dbg.ref.yaw_dot(i) = dbg_raw{i}.reference(11);
end