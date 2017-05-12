function command = readCommandRollPitchYawRateThrust( bag, topic  )

command_raw = bag.readAll(topic);
sz = length(command_raw);

command.t = zeros(1, sz);
command.i = zeros(1,sz);


for i=1:sz
    command.t(i) = timestampFromHeader(command_raw{i}.header);
    command.i(i) = command_raw{i}.header.seq;
    command.roll(:,i) = command_raw{i}.roll;
    command.pitch(:,i) = command_raw{i}.pitch;
    command.yaw_rate(:,i) = command_raw{i}.yaw_rate;
    command.thrust(:,i) = command_raw{i}.thrust;
end




end
