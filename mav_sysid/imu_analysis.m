%% Parameters
bag_name = 'YOUR BAG PATH HERE';
imu_topic = 'IMU TOPIC HERE EXAMPLE/mavros/imu/data';

%% Loading
path(path, '../read_bags');
path(path, '../helper_functions');

bag = ros.Bag(bag_name);
bag.info

imu = readImu(bag, imu_topic);
imu.t = imu.t - imu.t(1);

[publish_path,~,~] = fileparts(bag_name);

%% Lowpass the x axis IMU
RC = 0.016 * 2;
dt = 0.01;

Fpass = 5;
Fstop = 7.5;
Apass = 1.0;
Astop = 65;
Fs = 100;

d = designfilt('lowpassiir', ...
  'PassbandFrequency',Fpass,'StopbandFrequency',Fstop, ...
  'PassbandRipple',Apass,'StopbandAttenuation',Astop, ...
  'DesignMethod','butter','SampleRate',Fs);

[b, a] = butter(2, 5/100);
imu_filt = filter(b, a, imu.a(1, :));

%% Get the standard deviation for each second of operation.
std_a = zeros(floor(length(imu.t)/100), 1);
for i = 1:floor(length(imu.t)/100)
  std_a(i) = std(imu.a(1, (i-1)*100+1:i*100));
end

%% Figures
close all;
figure();
ax = axes;
plot(imu.t, imu.a(:,:), 'linewidth', 2);
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]');
title('Raw IMU data')
legend('a_x', 'a_y', 'a_z');
ylim([-10, 30]);
grid on;
ax.FontSize = 16;

figure();
ax = axes;
plot(imu.t, imu.w(:,:), 'linewidth', 2);
xlabel('Time [s]');
ylabel('Angular Rate [rad/s]');
title('Raw Gyro data');
legend('w_x', 'w_y', 'w_z');
ylim([-2, 2]);
grid on;
ax.FontSize = 16;
%%
figure()
ax = axes;
plot(imu.t, imu.a(1,:), '.');
hold on;
plot(imu.t, imu_filt, 'linewidth', 2);
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]');
title('Raw and Filtered IMU data')
legend('Raw', 'Filtered');
ylim([-5, 5]);
grid on;
ax.FontSize = 16;
%%
figure();
ax = axes;
plot(std_a);
ylim([0, 4]);
title('STD of Accel Data for Each Second')
xlabel('Time [s]');
ylabel('Stdev of Accelerometer Data');
grid on;
ax.FontSize = 16;

%%
figure()
ax = axes;
spectrogram(imu.a(1, :), 256, 250, 256, 100, 'yaxis');
title('Spectrogram!');
ax.FontSize = 16;