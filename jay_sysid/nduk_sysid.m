%% load bag files
clear all;
close all
clc;

%bag_name = '~/data/loon_sysid/2018-03-21-22-04-48.bag';
bag_name = '~/data/nduk_sysid/2018-11-19-15-13-11.bag';


%use data from t0 to t1
t0 = 10;
t1 = 130;


bag = ros.Bag(bag_name);
bag.info
%% read topics
imu_data = readImu(bag, '/nduk/mavros/imu/data');
imu_raw = readImu(bag, '/nduk/mavros/imu/data_raw');

% For old bags
% attitude_cmd = readPoseStamped(bag, '/jay/mavros/setpoint_attitude/attitude');
% thrust_cmd_raw = bag.readAll('/jay/mavros/setpoint_attitude/att_throttle');
% For new bags
% attitude_cmd = readAttitudeTarget(bag, '/jay/mavros/setpoint_raw/attitude');
% attitude_cmd.rpy = quat2rpy([attitude_cmd.q(4,:)', attitude_cmd.q(1:3,:)']');
% For newest bags
attitude_cmd = readCommandRollPitchYawRateThrust(bag, '/nduk/mavros/setpoint_raw/roll_pitch_yawrate_thrust');
%%
odometry = readOdometry(bag, '/nduk/msf_core/odometry');

current_reference = readCommandReference(bag, '/nduk/command/current_reference');

imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);

t_start = imu_raw.t(1);
imu_data.t = imu_data.t - t_start;
attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
imu_raw.t = imu_raw.t - t_start;
%% plot
figure(1);
ax = axes;
plot(imu_data.t, imu_data.rpy(1,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(1,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('roll [rad]');
title('roll angle');
legend('\phi imu', '\phi cmd');
grid on;
ax.FontSize = 16;

figure(2);
ax = axes;
plot(imu_data.t, imu_data.rpy(2,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(2,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('\theta [rad]');
title('pitch angle');
legend('\theta imu', '\theta cmd');
grid on;
ax.FontSize = 16;

figure(3);
ax = axes;
plot(imu_raw.t, imu_raw.a(3, :), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.thrust, '--', 'linewidth', 2);
xlabel('time');
ylabel('???');
title('Thrust');
legend('Acceleration', 'Commanded thrust');
grid on;
ax.FontSize = 16;

%% sysid

attitude_cmd.rpy_interp = zeros(size(imu_data.rpy));
attitude_cmd.rpy_interp(1,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(1,:), imu_data.t);
attitude_cmd.rpy_interp(2,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(2,:), imu_data.t);
attitude_cmd.rpy_interp(3,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(3,:), imu_data.t);

attitude_cmd.t = imu_data.t;

imu_data.t = imu_data.t(imu_data.t > t0 & imu_data.t < t1);
imu_data.rpy = imu_data.rpy(:, imu_data.t > t0 & imu_data.t < t1);

attitude_cmd.t = attitude_cmd.t(attitude_cmd.t > t0 & attitude_cmd.t < t1);
attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, attitude_cmd.t > t0 & attitude_cmd.t < t1);

dt = mean(diff(imu_data.t));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);

roll_tf = tfest( roll_data, 1, 0);
pitch_tf = tfest( pitch_data, 1, 0);
disp('======================');
disp('roll 1st order dynamics'); 
% roll_tf
disp(sprintf('roll_gain: %f\n', dcgain(roll_tf)));
disp(sprintf('roll_time_constant: %f\n', -1/pole(roll_tf)));
disp(sprintf('fit percentage: %f %%\n', roll_tf.Report.Fit.FitPercent));
disp('----------------------');
disp('pitch 1st order dynamics'); 
% pitch_tf
disp(sprintf('pitch_gain: %f\n', dcgain(pitch_tf)));
disp(sprintf('pitch_time_constant: %f\n', -1/pole(pitch_tf)));
disp(sprintf('fit percentage: %f %%\n', pitch_tf.Report.Fit.FitPercent));

%2nd order
roll_tf = tfest( roll_data, 2, 0);
pitch_tf = tfest( pitch_data, 2, 0);
disp('----------------------');
disp('roll 2st order dynamics:'); 
% roll_tf
[Wn, damping] = damp(roll_tf);
disp(sprintf('roll_gain: %f\n', dcgain(roll_tf)));
disp(sprintf('roll_damping_coef: %f\n', damping(1)));
disp(sprintf('roll_natural_freq: %f\n', Wn(1)));
disp(sprintf('fit percentage: %f %%\n', roll_tf.Report.Fit.FitPercent));

disp('----------------------');
disp('pitch 2st order dynamics:'); 
% pitch_tf
[Wn, damping] = damp(pitch_tf);
disp(sprintf('pitch_gain: %f\n', dcgain(pitch_tf)));
disp(sprintf('pitch_damping_coef: %f\n', damping(1)));
disp(sprintf('pitch_natural_freq: %f\n', Wn(1)));
disp(sprintf('fit percentage: %f %%\n', pitch_tf.Report.Fit.FitPercent));



%% 
