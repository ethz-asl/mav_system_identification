% 
% In order to run this script you need matlab_rosbag package
% https://github.com/bcharrow/matlab_rosbag (source)
% https://github.com/bcharrow/matlab_rosbag/releases (binary)
% 
% In case you face the follosing linking error
% matlab_rosbag-0.5.0-mac64/rosbag_wrapper.mexmaci64,
% 6): Symbol not found: __ZTISt16invalid_argument
% try this re-compiled binary
% https://cmu.app.box.com/s/9hs153nwa19uqvzboglkz7y84r6jzzxg    or
% https://dl.dropboxusercontent.com/u/12446150/matlab_rosbag-0.5.0-mac64_matlabR2015a.zip
% Tested platform: Mac EI Capitan 10.11.6 with MATLAB R2016a
%

clear all
path(path, '../read_bags');
path(path, '../helper_functions');

% two experiments are needed to validate the identification
bagfile_exp1 =  '../bags/falcon2_exp01.bag';
bagfile_exp2 =  '../bags/falcon2_exp02.bag';


topic_imu = '/falcon2/dji_sdk/imu';
topic_vcdata = '/falcon2/dji_sdk/vc_cmd';

bag1 = ros.Bag(bagfile_exp1);
bag2 = ros.Bag(bagfile_exp2);
%%
% *First experriment info:*
bag1.info

%%
% *Second experiment info:*
bag2.info

%% Prepare datasets
Experiment1.IMU = readImu(bag1, topic_imu);
Experiment1.RCData = readCommnadRollPitchYawRateThrust(bag1, topic_vcdata);

Experiment2.IMU = readImu(bag2, topic_imu);
Experiment2.RCData = readCommnadRollPitchYawRateThrust(bag2, topic_vcdata);


% Write the quaternions properly

Experiment1.IMU.q = [Experiment1.IMU.q(4,:); Experiment1.IMU.q(1,:); ...
    Experiment1.IMU.q(2,:); Experiment1.IMU.q(3,:)];
Experiment2.IMU.q = [Experiment2.IMU.q(4,:); Experiment2.IMU.q(1,:); ...
    Experiment2.IMU.q(2,:); Experiment2.IMU.q(3,:)];

%time from 0
Experiment1.RCData.t = Experiment1.RCData.t - Experiment1.RCData.t(1);
Experiment1.IMU.t = Experiment1.IMU.t - Experiment1.IMU.t(1);

Experiment2.RCData.t = Experiment2.RCData.t - Experiment2.RCData.t(1);
Experiment2.IMU.t = Experiment2.IMU.t - Experiment2.IMU.t(1);

%========================================================
%RCDATA interpolation with respect to IMU
%Exp1
%========================================================

cmd_roll_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.roll,Experiment1.IMU.t,'spline');
cmd_pitch_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.pitch,Experiment1.IMU.t,'spline');
cmd_yawrate_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.yaw_rate,Experiment1.IMU.t,'spline');
cmd_thrust_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.thrust(3,:),Experiment1.IMU.t,'spline');

Experiment1.IMU.q=[Experiment1.IMU.q(1,:);Experiment1.IMU.q(2,:);Experiment1.IMU.q(3,:);Experiment1.IMU.q(4,:)];

Experiment1.RCData.roll=cmd_roll_intp;
Experiment1.RCData.pitch=cmd_pitch_intp;
Experiment1.RCData.yaw_rate=cmd_yawrate_intp;
Experiment1.RCData.verti_vel=cmd_thrust_intp;

Experiment1.RCData.t=zeros(1,size(cmd_roll_intp,2));
Experiment1.RCData.t=Experiment1.IMU.t;

%Exp2

cmd_roll_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.roll,Experiment2.IMU.t,'spline');
cmd_pitch_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.pitch,Experiment2.IMU.t,'spline');
cmd_yawrate_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.yaw_rate,Experiment2.IMU.t,'spline');
cmd_thrust_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.thrust(3,:),Experiment2.IMU.t,'spline');

Experiment2.IMU.q=[Experiment2.IMU.q(1,:);Experiment2.IMU.q(2,:);Experiment2.IMU.q(3,:);Experiment2.IMU.q(4,:)];

Experiment2.RCData.roll=cmd_roll_intp;
Experiment2.RCData.pitch=cmd_pitch_intp;
Experiment2.RCData.yaw_rate=cmd_yawrate_intp;
Experiment2.RCData.verti_vel=cmd_thrust_intp;

Experiment2.RCData.t=zeros(1,size(cmd_roll_intp,2));
Experiment2.RCData.t=Experiment2.IMU.t;


Experiment1.rpy_imu = quat2rpy(Experiment1.IMU.q);
Experiment2.rpy_imu = quat2rpy(Experiment2.IMU.q);

% For DJI M100 platform.
%Please have a look "DJI_M100_regression for more detail.
k_pitch = 0.000844; 
k_roll  = 0.000865;
k_thrust = 0.0019965;
k_yaw = 0.002235;

%DJI vc channel, 1=pitch, 2=roll, 3=vertical velocity, 4=yaw_rate.

Experiment1.roll_cmd    = (Experiment1.RCData.roll-1024)...
    *k_roll;
Experiment1.pitch_cmd   = (Experiment1.RCData.pitch-1024)...
    *k_pitch;
Experiment1.yaw_cmd = -(Experiment1.RCData.yaw_rate-1024)*k_yaw;
Experiment1.thrust_cmd  = (Experiment1.RCData.verti_vel-1024)...
    *k_thrust;%stick velocity command.


Experiment2.roll_cmd    = (Experiment2.RCData.roll-1024)...
    *k_pitch;
Experiment2.pitch_cmd   = (Experiment2.RCData.pitch-1024)...
    *k_roll;
Experiment2.yaw_cmd = -(Experiment2.RCData.yaw_rate-1024)*k_yaw;
Experiment2.thrust_cmd  = (Experiment2.RCData.verti_vel-1024)...
    *k_thrust;

figure;
% *Plot attitude from experiment 2*
title('Experiment 1 Data');
plot(Experiment1.IMU.t, Experiment1.IMU.w(3,:)*180/pi, ...
    Experiment1.RCData.t, medfilt1(Experiment1.yaw_cmd*180/pi,20), ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('yaw rate [deg]/s');
title('yaw rate from IMU');


%% Identification of roll system

%Control parameters

delay=[]; NaN; % [] menas no time delay or NaN for enabling delay estimation.
np=2; % 1 or 2 for the order of dynamics system.
nz = 0;

%% The length of data may vary.
Experiment1.t = (Experiment1.IMU.t + Experiment1.RCData.t)/2;
Experiment1.u1 = Experiment1.yaw_cmd;
Experiment1.y1 = Experiment1.IMU.w(3,:);
Experiment1.Ts = mean(diff(Experiment1.IMU.t));

%%
% Choose valid data range.
Experiment1.u1 = Experiment1.u1(Experiment1.t>63 & ...
    Experiment1.t < 77);
Experiment1.y1 = Experiment1.y1(Experiment1.t>63 &...
    Experiment1.t < 77);

yawrate_data1 = iddata(Experiment1.y1',Experiment1.u1',Experiment1.Ts, ...
    'ExperimentName', 'Falcon2_sysID_yaw1', 'InputName','yawrate_{cmd}', ...
    'OutputName','yawrate', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

yawrate_data1 = detrend(yawrate_data1);


%% The length of data may vary.
Experiment2.t = (Experiment2.IMU.t + Experiment2.RCData.t)/2;
Experiment2.u1 = Experiment2.yaw_cmd;
Experiment2.y1 = Experiment2.IMU.w(3,:);
Experiment2.Ts = mean(diff(Experiment2.IMU.t));


%%
% Choose valid data range.
Experiment2.u1 = Experiment2.u1(Experiment2.t>50 & ...
    Experiment2.t < 63);
Experiment2.y1 = Experiment2.y1(Experiment2.t>50 &...
    Experiment2.t < 63);

yawrate_data2 = iddata(Experiment2.y1',Experiment2.u1',Experiment2.Ts, ...
    'ExperimentName', 'Falcon2_sysID_yaw2', 'InputName','yawrate_{cmd}', ...
    'OutputName','yawrate', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

%Generate model using Experiment1 and validate the model with Experiment2
yawrate_estimated_tf1 = tfest(yawrate_data1,np, nz);

[~, fit1, ~] = compare(yawrate_data2, yawrate_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
yawrate_estimated_tf2 = tfest(yawrate_data2,np, nz);

[~, fit2, ~] = compare(yawrate_data1, yawrate_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    yawrate_estimated_tf = yawrate_estimated_tf1;
    disp('The yawrate model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(yawrate_data2, yawrate_estimated_tf1);
    disp(strcat('The yawrate model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    yawrate_estimated_tf = yawrate_estimated_tf2;
    disp('The yawrate model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(yawrate_data1, yawrate_estimated_tf2);
    disp(strcat('The yawrate model fits the validation data with **',...
        num2str(fit2), '** %'));
end

%%% Estimated Transfer Functions

disp('yawrate estimated transfer function is: ');
tf(yawrate_estimated_tf)
yawrate_params=getpvec(yawrate_estimated_tf);
if(np==1)
    yawrate_gain=yawrate_params(1)/yawrate_params(2);
    yawrate_tau=1/yawrate_params(2);
    fprintf('yawrate gain=%.3f, tau=%.3f\n',yawrate_gain,yawrate_tau);
elseif(np==2)
    yawrate_omega=sqrt(yawrate_params(3));
    yawrate_gain=yawrate_params(1)/yawrate_params(3);
    yawrate_damping=yawrate_params(2)/(2*yawrate_omega);
    fprintf('yawrate omega=%.3f, gain=%.3f damping=%.3f\n',yawrate_omega,yawrate_gain,yawrate_damping);
end


figure('Name','System analysis (yawrate)');
subplot(311);
bode(yawrate_estimated_tf); grid;
title('yawrate bode plot');

subplot(312);
rlocusplot(yawrate_estimated_tf); grid;
title('yawrate RootLucas plot');

subplot(313);
step(yawrate_estimated_tf); grid;
title('yawrate step response plot');




