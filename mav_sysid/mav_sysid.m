%% user set params

bag_name = 'YOUR BAG PATH HERE';
imu_topic = 'IMU TOPIC HERE EXAMPLE/mavros/imu/data';
control_topic = 'CONTROL TOPIC HERE EXAMPLE/mavros/setpoint_raw/roll_pitch_yawrate_thrust';

sys_id_start_time_s = 10;
sys_id_end_time_s = 150;

% parameters that must be provided 
system_mass_kg = 2.5;
linear_drag_coefficients = [0.02, 0.02, 0]; %default values work for most systems
yaw_gain = 1.0; %default value works for most systems
yaw_damping = 0.95; %default value works for most systems
yaw_omega = 5.0; %default value works for most systems

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

close all;
clc;

bag = ros.Bag(bag_name);
bag.info

imu_data = readImu(bag, imu_topic);
attitude_cmd = readCommandRollPitchYawRateThrust(bag, control_topic);
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
plot(imu_data.t, vecnorm(imu_data.a), 'linewidth', 2);
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

attitude_cmd.thrust_interp = zeros(size(imu_data.rpy));
attitude_cmd.thrust_interp = interp1(attitude_cmd.t, attitude_cmd.thrust(3,:), imu_data.t);

attitude_cmd.t = imu_data.t;

imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
imu_data.rpy = imu_data.rpy(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
imu_data.a = imu_data.a(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);

attitude_cmd.t = attitude_cmd.t(attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
attitude_cmd.thrust_interp = attitude_cmd.thrust_interp(:, attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);

dt = mean(diff(imu_data.t));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);

roll_tf = tfest( roll_data, 1, 0);
pitch_tf = tfest( pitch_data, 1, 0);

disp('=====================================================================');
disp('1st order dynamics'); 
fprintf('pitch fit percentage: %f%%\n', pitch_tf.Report.Fit.FitPercent);
fprintf('roll fit percentage: %f%%\n\n', roll_tf.Report.Fit.FitPercent);
disp('---------------------------------------------------------------------');
disp('Copy the following into your nonlinear_mpc.yaml file')
disp('---------------------------------------------------------------------');
disp('# Controller Parameters:');
fprintf('mass: %f\n',system_mass_kg);
fprintf('roll_time_constant: %f\n', -1/pole(roll_tf));
fprintf('roll_gain: %f\n', dcgain(roll_tf));
fprintf('pitch_time_constant: %f\n', -1/pole(pitch_tf));
fprintf('pitch_gain: %f\n', dcgain(pitch_tf));
fprintf('linear_drag_coefficients: [%f, %f, %f]\n', linear_drag_coefficients(1),linear_drag_coefficients(2),linear_drag_coefficients(3));
fprintf('yaw_gain: %f\n', yaw_gain);
disp('---------------------------------------------------------------------');

%2nd order
roll_tf = tfest( roll_data, 2, 0);
pitch_tf = tfest( pitch_data, 2, 0);

[roll_Wn, roll_damping] = damp(roll_tf);
[pitch_Wn, pitch_damping] = damp(pitch_tf);

disp('=====================================================================');
disp('2nd order dynamics'); 
fprintf('pitch fit percentage: %f%%\n', pitch_tf.Report.Fit.FitPercent);
fprintf('roll fit percentage: %f%%\n\n', roll_tf.Report.Fit.FitPercent);
disp('---------------------------------------------------------------------');
disp('Copy the following into your disturbance_observer.yaml file');
disp('---------------------------------------------------------------------');
disp('  #model from system identification (2nd order attitude model)');
fprintf('  drag_coefficients: [%f, %f, %f]\n', linear_drag_coefficients(1),linear_drag_coefficients(2),linear_drag_coefficients(3));
fprintf('  roll_gain: %f\n', dcgain(roll_tf));
fprintf('  roll_damping: %f\n', roll_damping(1));
fprintf('  roll_omega: %f\n', roll_Wn(1));
fprintf('  pitch_gain: %f\n', dcgain(pitch_tf));
fprintf('  pitch_damping: %f\n', pitch_damping(1));
fprintf('  pitch_omega: %f\n', pitch_Wn(1));
fprintf('  yaw_gain: %f\n', yaw_gain);
fprintf('  yaw_damping: %f\n', yaw_damping);
fprintf('  yaw_omega: %f\n', yaw_omega);
disp('---------------------------------------------------------------------');

%thrust
acc_thrust_ratio = (vecnorm(imu_data.a)-9.81) ./ attitude_cmd.thrust_interp;
acc_thrust_ratio = median(acc_thrust_ratio(acc_thrust_ratio > 0.0)) / system_mass_kg;

disp('=====================================================================');
disp('pixhawk thrust settings'); 
disp('---------------------------------------------------------------------');
disp('Copy the following into your px4_config.yaml file')
disp('---------------------------------------------------------------------');
disp('# setpoint_raw');
disp('setpoint_raw:');
fprintf('  thrust_scaling_factor: %f\n',acc_thrust_ratio);
fprintf('  system_mass_kg: %f\n', system_mass_kg);
fprintf('  yaw_rate_scaling_factor: %f\n', yaw_gain);
disp('---------------------------------------------------------------------');


%% 
