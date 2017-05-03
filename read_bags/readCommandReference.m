function commandTrajectoryArray = readCommandReference(bag, topic)

command_trajectory_array_raw = bag.readAll(topic);

sz = length(command_trajectory_array_raw);

commandTrajectoryArray.t = zeros(1, sz);
commandTrajectoryArray.i = zeros(1, sz);
commandTrajectoryArray.p = zeros(3, sz);
commandTrajectoryArray.q = zeros(4, sz);
commandTrajectoryArray.v = zeros(3, sz);
commandTrajectoryArray.w = zeros(3, sz);
commandTrajectoryArray.a = zeros(3, sz);
commandTrajectoryArray.alpha = zeros(3, sz);
commandTrajectoryArray.time_from_start = zeros(1, sz);


for i=1:sz
    commandTrajectoryArray.t(i) = timestampFromHeader(command_trajectory_array_raw{i}.header);
    commandTrajectoryArray.i(i) = command_trajectory_array_raw{i}.header.seq;
    j = 1;
    commandTrajectoryArray.p(:, i) = command_trajectory_array_raw{i}.points(j).transforms.translation(:);
    commandTrajectoryArray.q(:, i) = command_trajectory_array_raw{i}.points(j).transforms.rotation(:);
    commandTrajectoryArray.v(:, i) = command_trajectory_array_raw{i}.points(j).velocities.linear(:);
    commandTrajectoryArray.w(:, i) = command_trajectory_array_raw{i}.points(j).velocities.angular(:);
    commandTrajectoryArray.a(:, i) = command_trajectory_array_raw{i}.points(j).accelerations.linear(:);
    commandTrajectoryArray.alpha(:, i) = command_trajectory_array_raw{i}.points(j).accelerations.angular(:);
    commandTrajectoryArray.time_from_start(i) = double(command_trajectory_array_raw{i}.points(j).time_from_start.sec) + double(command_trajectory_array_raw{i}.points(j).time_from_start.nsec)*1e-9;
end