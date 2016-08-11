function ms = readMotorSpeed(bag, topic)

ms_raw = bag.readAll(topic);

sz = length(ms_raw);

ms.motor_speed = zeros(6, sz);

for i=1:sz
   ms.t(i) = timestampFromHeader(ms_raw{i}.header);
   ms.motor_speed(:,i) = ms_raw{i}.motor_speed;
end