function wind_speed = readWindSpeed(bag, topic)

wind_speed_raw = bag.readAll(topic);

sz = length(wind_speed_raw);

wind_speed.wind_speed = zeros(3, sz);

for i=1:sz
   wind_speed.t(i) = timestampFromHeader(wind_speed_raw{i}.header);
   wind_speed.wind_speed(:,i) = wind_speed_raw{i}.velocity;
end