function vel_out = readDJI_Velocity( bag, topic )

vel_raw = bag.readAll(topic);
sz = length(vel_raw);

vel_out.t = zeros(1, sz);
vel_out.i = zeros(1,sz);
vel_out.vel = zeros(3, sz);

for i=1:sz
    vel_out.t(i) = timestampFromHeader(vel_raw{i}.header);
    vel_out.i(i) = vel_raw{i}.header.seq;
    vel_out.vx(i) = vel_raw{i}.vx;
    vel_out.vy(i) = vel_raw{i}.vy;
    vel_out.vz(i) = vel_raw{i}.vz;
end

