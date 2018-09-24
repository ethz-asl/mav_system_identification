%% user set params

bag_name = 'YOUR BAG PATH HERE';
imu_topic = 'IMU TOPIC HERE EXAMPLE/mavros/imu/data';
control_topic = 'CONTROL TOPIC HERE EXAMPLE/mavros/setpoint_raw/roll_pitch_yawrate_thrust';

sys_id_start_time_s = 10;
sys_id_end_time_s = 150;

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

close all;
clc;

bag = ros.Bag(bag_name);
bag.info

imu_data = readImu(bag, '/nduk/mavros/imu/data');
attitude_cmd = readCommandRollPitchYawRateThrust(bag, '/nduk/mavros/setpoint_raw/roll_pitch_yawrate_thrust');
%%

imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);

t_start = imu_data.t(1);
imu_data.t = imu_data.t - t_start;
attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
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
plot(imu_data.t, imu_data.a(3, :), 'linewidth', 2);
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

imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
imu_data.rpy = imu_data.rpy(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);

attitude_cmd.t = attitude_cmd.t(attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);

dt = mean(diff(imu_data.t));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);

roll_tf = tfest( roll_data, 1, 0);
pitch_tf = tfest( pitch_data, 1, 0);
disp('======================');
disp('roll 1st order dynamics'); 
% roll_tf
fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
fprintf('roll_time_constant: %f\n\n', -1/pole(roll_tf));
fprintf('fit percentage: %f %%\n', roll_tf.Report.Fit.FitPercent);
disp('----------------------');
disp('pitch 1st order dynamics'); 
% pitch_tf
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
fprintf('pitch_time_constant: %f\n\n', -1/pole(pitch_tf));
fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);

%2nd order
roll_tf = tfest( roll_data, 2, 0);
pitch_tf = tfest( pitch_data, 2, 0);
disp('----------------------');
disp('roll 2st order dynamics:'); 
% roll_tf
[Wn, damping] = damp(roll_tf);
fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
fprintf('roll_damping_coef: %f\n\n', damping(1));
fprintf('roll_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', roll_tf.Report.Fit.FitPercent);

disp('----------------------');
disp('pitch 2st order dynamics:'); 
% pitch_tf
[Wn, damping] = damp(pitch_tf);
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
fprintf('pitch_damping_coef: %f\n\n', damping(1));
fprintf('pitch_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);



%% 
