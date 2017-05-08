%% Parameters
bag_name = '~/data/jay/comparison/jay_blue/2017-05-03-13-24-26.bag';
%bag_name = '~/data/jay/comparison/jay_foam/2017-05-03-14-40-48.bag';
%bag_name = '~/data/jay/comparison/jay_gel/2017-05-03-18-50-36.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping/2017-05-04-15-54-08.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping_5hz/2017-05-04-16-00-13.bag';
%bag_name = '~/data/jay/comparison/imu_double_damping_5hz/2017-05-04-16-05-34.bag';
%bag_name = '~/data/jay/comparison/imu_blue_5hz/2017-05-04-18-00-37.bag';
%bag_name = '~/data/jay/comparison/imu_butter_5hz/2017-05-05-14-11-25.bag';
bag_name = '~/data/jay/comparison/imu_butter_real_5hz/2017-05-05-16-49-00.bag';

mav_name = 'jay';

%bag_name = '~/data/jay/comparison/ibis/2017-05-03-13-50-27.bag';
%mav_name = 'ibis';

%bag_name = '~/data/euroc_datasets/V1_01_easy.bag';
%mav_name = 'euroc';


%% Loading
bag = ros.Bag(bag_name);
bag.info

if (strcmp(mav_name, 'jay') == 1)
  imu_topic =  '/jay/mavros/imu/data_raw';
elseif (strcmp(mav_name, 'ibis') == 1)
  imu_topic = '/ibis/imu';
else
  imu_topic = '/fcu/imu';
end

imu = readImu(bag, imu_topic);
imu.t = imu.t - imu.t(1);

%% Lowpass the x axis IMU
RC = 0.016 * 2;
dt = 0.01;
%imu_filt = low_pass(imu.a(1, :), dt, RC);
%imu_filt = medfilt1(imu.a(1, :), 100);
%imu_filt = conv(imu.a(1, :), ones(1, 20)/20, 'same');

Fpass = 5;
Fstop = 7.5;
Apass = 1.0;
Astop = 65;
Fs = 100;

d = designfilt('lowpassiir', ...
  'PassbandFrequency',Fpass,'StopbandFrequency',Fstop, ...
  'PassbandRipple',Apass,'StopbandAttenuation',Astop, ...
  'DesignMethod','butter','SampleRate',Fs);
%d = designfilt('lowpassiir', 'FilterOrder', 2, 'StopbandFrequency', ...
%               Fstop, 'StopbandAttenuation', Astop, 'SampleRate', Fs, ...
%               'DesignMethod', 'cheby2');

%imu_filt = filter(d, imu.a(1, :));

[b, a] = butter(2, 5/100);
imu_filt = filter(b, a, imu.a(1, :));

%% Get the standard deviation for each second of operation.
std_a = zeros(floor(length(imu.t)/100), 1);
for i = 1:floor(length(imu.t)/100)
  std_a(i) = std(imu.a(1, (i-1)*100+1:i*100));
end

%% Figures
%close all;
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
%spectrogram(imu_filt, 256, 250, 256, 100, 'yaxis');
title('Spectrogram!');
ax.FontSize = 16;

%%
%fvtool(d)

