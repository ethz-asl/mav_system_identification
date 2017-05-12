%% Parameters
bag_name = '~/data/jay/comparison/jay_blue/2017-05-03-13-24-26.bag';
%bag_name = '~/data/jay/comparison/jay_foam/2017-05-03-14-40-48.bag';
%bag_name = '~/data/jay/comparison/jay_gel/2017-05-03-18-50-36.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping/2017-05-04-15-54-08.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping_5hz/2017-05-04-16-00-13.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping_5hz/2017-05-04-16-05-34.bag';
%bag_name = '~/data/jay/comparison/imu_blue_5hz/2017-05-04-18-00-37.bag';
%bag_name = '~/data/jay/comparison/imu_butter_5hz/2017-05-05-14-11-25.bag';
%bag_name = '~/data/jay/comparison/imu_butter_real_5hz/2017-05-05-16-49-00.bag';
%bag_name = '~/data/jay/comparison/imu_butter_10hz/2017-05-05-17-07-01.bag';

mav_name = 'jay';

%% Load
bag = ros.Bag(bag_name);
bag.info

imu = readImu(bag, '/jay/mavros/imu/data_raw');
odometry = readOdometry(bag, '/jay/msf_core/odometry');
vicon_odometry = readOdometry(bag, '/jay/vrpn_client/estimated_odometry');

odometry.rpy = quat2rpy([odometry.q(4,:)', odometry.q(1:3,:)']');
vicon_odometry.rpy = quat2rpy([vicon_odometry.q(4,:)', vicon_odometry.q(1:3,:)']');

t_start = imu.t(1);
imu.t = imu.t - t_start;
odometry.t = odometry.t - t_start;
vicon_odometry.t = vicon_odometry.t - t_start;

[publish_path,~,~] = fileparts(bag_name);

%% Get a velocity estimate by finite difference out of vicon
smoothing_parameter=0.9999;
pp = csaps(vicon_odometry.t, vicon_odometry.p, smoothing_parameter);
dpp = fnder(pp);
velocity = fnval(dpp, vicon_odometry.t);

B_v_WB = zeros(size(velocity));
for i=1:length(velocity)
  B_v_WB(:,i) = quat2rot(vicon_odometry.q([4 1 2 3],i))'*velocity(:,i);
end

%% Plot The Things
figure();
ax = axes;
plot(odometry.t, odometry.v(:, :), 'linewidth', 2);
hold on;
plot(vicon_odometry.t, B_v_WB, '--', 'linewidth', 2);
xlabel('time');
ylabel('Velocity [m/s]');
title('Velocities');
legend('Odometry MSF x', 'Odometry MSF y', 'Odometry MSF z', 'Odometry Vicon x', 'Odometry Vicon y', 'Odometry Vicon z', 'Location', 'NorthWest');
grid on;
ax.FontSize = 16;

figure();
ax = axes;
plot(odometry.t, odometry.p(:, :), 'linewidth', 2);
hold on;
plot(vicon_odometry.t, vicon_odometry.p(:,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('Positions [m]');
title('Positions');
legend('Odometry MSF x', 'Odometry MSF y', 'Odometry MSF z', 'Odometry Vicon x', 'Odometry Vicon y', 'Odometry Vicon z', 'Location', 'NorthWest');
grid on;
ax.FontSize = 16;