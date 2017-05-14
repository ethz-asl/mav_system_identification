clear all
path(path, './read_bags');
path(path, './helper_functions');
path(path, '/Users/Inkyu/Research/ETH_Postdoc/Flourish/FlourishDev/DJI/DJI_sysID_again/src');


% two experiments are needed to validate the identification
bagfile_exp1 =  './bags/falcon2_exp01_again_again.bag';
bagfile_exp2 =  './bags/falcon2_exp02_again_again.bag';



topic_imu = '/falcon2/dji_sdk/imu';
topic_rcdata = '/falcon2/dji_sdk/vc_cmd';
topic_vicon = '/falcon2/vrpn_client/estimated_odometry';

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
Experiment1.RCData = readCommnadRollPitchYawRateThrust(bag1, topic_rcdata);
Experiment1.Vicon = readOdometry(bag1, topic_vicon);


Experiment2.IMU = readImu(bag2, topic_imu);
Experiment2.RCData = readCommnadRollPitchYawRateThrust(bag2, topic_rcdata);
Experiment2.Vicon = readOdometry(bag2, topic_vicon);

% Write the quaternions from VICON properly

Experiment1.Vicon.q = [Experiment1.Vicon.q(4,:); Experiment1.Vicon.q(1,:);...
    Experiment1.Vicon.q(2,:); Experiment1.Vicon.q(3,:)];
Experiment1.IMU.q = [Experiment1.IMU.q(4,:); Experiment1.IMU.q(1,:); ...
    Experiment1.IMU.q(2,:); Experiment1.IMU.q(3,:)];
Experiment2.Vicon.q = [Experiment2.Vicon.q(4,:); Experiment2.Vicon.q(1,:);...
    Experiment2.Vicon.q(2,:); Experiment2.Vicon.q(3,:)];
Experiment2.IMU.q = [Experiment2.IMU.q(4,:); Experiment2.IMU.q(1,:); ...
    Experiment2.IMU.q(2,:); Experiment2.IMU.q(3,:)];


Experiment1.rpy = quat2rpy(Experiment1.Vicon.q);
Experiment1.rpy_imu = quat2rpy(Experiment1.IMU.q);

Experiment2.rpy = quat2rpy(Experiment2.Vicon.q);
Experiment2.rpy_imu = quat2rpy(Experiment2.IMU.q);


%Experiment1.rpy(2,:) = -Experiment1.rpy(2,:);
%Experiment2.rpy(2,:) = -Experiment2.rpy(2,:);


%time from 0
Experiment1.Vicon.t = Experiment1.Vicon.t - Experiment1.Vicon.t(1);
Experiment1.RCData.t = Experiment1.RCData.t - Experiment1.RCData.t(1);
Experiment1.IMU.t = Experiment1.IMU.t - Experiment1.IMU.t(1);

Experiment2.Vicon.t = Experiment2.Vicon.t - Experiment2.Vicon.t(1);
Experiment2.RCData.t = Experiment2.RCData.t - Experiment2.RCData.t(1);
Experiment2.IMU.t = Experiment2.IMU.t - Experiment2.IMU.t(1);


figure(1);
title('Experiment 2 Data');
plot(Experiment2.Vicon.t, Experiment2.Vicon.v(3,:), ...
    Experiment2.RCData.t, (Experiment2.RCData.thrust(3,:)-1024)*0.002649, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('m/s');
title('dz from vicon');

%========================================================
%IMU, RCDATA interpolation with respect to vicon
%Exp1
%========================================================

imu_q1_intp=interp1(Experiment1.IMU.t,Experiment1.IMU.q(1,:),Experiment1.Vicon.t,'spline');
imu_q2_intp=interp1(Experiment1.IMU.t,Experiment1.IMU.q(2,:),Experiment1.Vicon.t,'spline');
imu_q3_intp=interp1(Experiment1.IMU.t,Experiment1.IMU.q(3,:),Experiment1.Vicon.t,'spline');
imu_q4_intp=interp1(Experiment1.IMU.t,Experiment1.IMU.q(4,:),Experiment1.Vicon.t,'spline');

cmd_ch1_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.pitch,Experiment1.Vicon.t,'spline');
cmd_ch2_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.roll,Experiment1.Vicon.t,'spline');
cmd_ch3_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.yaw_rate,Experiment1.Vicon.t,'spline');
cmd_ch4_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.thrust(3,:),Experiment1.Vicon.t,'spline');

Experiment1.IMU.q=zeros(4,size(imu_q1_intp,2));
Experiment1.IMU.q=[imu_q1_intp;imu_q2_intp;imu_q3_intp;imu_q4_intp];
Experiment1.IMU.t=zeros(1,size(imu_q1_intp,2));
Experiment1.IMU.t=Experiment1.Vicon.t;

Experiment1.RCData.channel=zeros(4,size(cmd_ch1_intp,2));
Experiment1.RCData.channel=[cmd_ch1_intp;cmd_ch2_intp;cmd_ch3_intp;cmd_ch4_intp];
Experiment1.RCData.t=zeros(1,size(cmd_ch1_intp,2));
Experiment1.RCData.t=Experiment1.Vicon.t;

%Exp2
imu_q1_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(1,:),Experiment2.Vicon.t,'spline');
imu_q2_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(2,:),Experiment2.Vicon.t,'spline');
imu_q3_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(3,:),Experiment2.Vicon.t,'spline');
imu_q4_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(4,:),Experiment2.Vicon.t,'spline');

cmd_ch1_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.pitch,Experiment2.Vicon.t,'spline');
cmd_ch2_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.roll,Experiment2.Vicon.t,'spline');
cmd_ch3_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.yaw_rate,Experiment2.Vicon.t,'spline');
cmd_ch4_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.thrust(3,:),Experiment2.Vicon.t,'spline');

Experiment2.IMU.q=zeros(4,size(imu_q1_intp,2));
Experiment2.IMU.q=[imu_q1_intp;imu_q2_intp;imu_q3_intp;imu_q4_intp];
Experiment2.IMU.t=zeros(1,size(imu_q1_intp,2));
Experiment2.IMU.t=Experiment2.Vicon.t;

Experiment2.RCData.channel=zeros(4,size(cmd_ch1_intp,2));
Experiment2.RCData.channel=[cmd_ch1_intp;cmd_ch2_intp;cmd_ch3_intp;cmd_ch4_intp];
Experiment2.RCData.t=zeros(1,size(cmd_ch1_intp,2));
Experiment2.RCData.t=Experiment2.Vicon.t;


%=================================
% commands
% This conversion is not fixed, change it accordingly
% This needs to be adjusted depending on a scale of commands.
% For Asctech platforms.
%=================================

% k_pitch_roll = (51.2*pi/180)/2047;
% k_thrust = 100/4095;
% 
% Experiment1.roll_cmd    = (Experiment1.RCData.channel(2,:)-2047)...
%     *k_pitch_roll;
% Experiment1.pitch_cmd   = (Experiment1.RCData.channel(1,:)-2047)...
%     *k_pitch_roll;
% Experiment1.thrust_cmd  = Experiment1.RCData.channel(3,:)...
%     *k_thrust;
% 
% Experiment2.roll_cmd    = (Experiment2.RCData.channel(2,:)-2047)...
%     *k_pitch_roll;
% Experiment2.pitch_cmd   = (Experiment2.RCData.channel(1,:)-2047)...
%     *k_pitch_roll;
% Experiment2.thrust_cmd  = Experiment2.RCData.channel(3,:)*k_thrust;

% For DJI M100 platform.
%Please have a look "DJI_M100_regression for more detail.
k_pitch = 0.000844; 
k_roll  = 0.000865;
k_thrust = 0.002649;
%DJI vc channel, 1=pitch, 2=roll, 3=thrust, 4=yaw_rate.

Experiment1.roll_cmd    = (Experiment1.RCData.channel(2,:)-1024)...
    *k_roll;
Experiment1.pitch_cmd   = (Experiment1.RCData.channel(1,:)-1024)...
    *k_pitch;
Experiment1.thrust_cmd  = (Experiment1.RCData.channel(3,:)-1024)...
    *k_thrust; %stick velocity command.

Experiment2.roll_cmd    = (Experiment2.RCData.channel(2,:)-1024)...
    *k_pitch;
Experiment2.pitch_cmd   = (Experiment2.RCData.channel(1,:)-1024)...
    *k_roll;
Experiment2.thrust_cmd  = (Experiment2.RCData.channel(3,:)-1024)*k_thrust;



%%
% *Plot position from experiment 1*
close all;
figure(1);
title('Experiment 1 Data');
subplot(3,1,1);
plot(Experiment1.Vicon.t, Experiment1.Vicon.p(1,:), 'linewidth', 2);
xlabel('time');
ylabel('x [m]');
title('x from vicon');

subplot(3,1,2);
plot(Experiment1.Vicon.t, Experiment1.Vicon.p(2,:), 'linewidth', 2);
xlabel('time');
ylabel('y [m]');
title('y from vicon');

subplot(3,1,3);
plot(Experiment1.Vicon.t, Experiment1.Vicon.p(3,:), 'linewidth', 2);
xlabel('time');
ylabel('z [m]');
title('z from vicon');

%%
% *Plot dz from experiment 1*
figure(2);
title('Experiment 1 Data');
subplot(1,1,1);
plot(Experiment1.Vicon.t, Experiment1.Vicon.v(3,:), ...
    Experiment1.RCData.t, Experiment1.thrust_cmd, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('m/s');
title('dz from vicon');

%%
% *Plot position from experiment 2*
figure(3);
title('Experiment 2 Data');
subplot(3,1,1);
plot(Experiment2.Vicon.t, Experiment2.Vicon.p(1,:), 'linewidth', 2);
xlabel('time');
ylabel('x [m]');
title('x from vicon');

subplot(3,1,2);
plot(Experiment2.Vicon.t, Experiment2.Vicon.p(2,:), 'linewidth', 2);
xlabel('time');
ylabel('y [m]');
title('y from vicon');

subplot(3,1,3);
plot(Experiment2.Vicon.t, Experiment2.Vicon.p(3,:), 'linewidth', 2);
xlabel('time');
ylabel('z [m]');
title('z from vicon');

figure(4);
title('Experiment 2 Data');
subplot(1,1,1);
plot(Experiment2.Vicon.t, Experiment2.Vicon.v(3,:), ...
    Experiment2.RCData.t, Experiment2.thrust_cmd, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('m/s');
title('dz from vicon');


%% Identification of roll system
%% The length of data may vary.
Experiment1.t = (Experiment1.Vicon.t + Experiment1.RCData.t)/2;
Experiment1.u1 = Experiment1.thrust_cmd;
Experiment1.y1 = Experiment1.Vicon.v(3,:);
Experiment1.Ts = mean(diff(Experiment1.Vicon.t));

%%
% *get rid of first and last 10 seconds (to remove ground and transient effects)*
Experiment1.u1 = Experiment1.u1(Experiment1.t>10 & ...
    Experiment1.t < Experiment1.t(end)-10);
Experiment1.y1 = Experiment1.y1(Experiment1.t>10 &...
    Experiment1.t < Experiment1.t(end)-10);
%Experiment1.t = Experiment1.t(Experiment1.t>10 & Experiment1.t < Experiment1.t(end)-10);

thrust_data1 = iddata(Experiment1.y1',Experiment1.u1',Experiment1.Ts, ...
    'ExperimentName', 'FireFlySysID_1', 'InputName','thrust_{cmd}', ...
    'OutputName','thrust', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

thrust_data1 = detrend(thrust_data1);


%% The length of data may vary.
Experiment2.t = (Experiment2.Vicon.t + Experiment2.RCData.t)/2;
Experiment2.u1 = Experiment2.thrust_cmd;
Experiment2.y1 = Experiment2.Vicon.v(3,:);
Experiment2.Ts = mean(diff(Experiment2.Vicon.t));


%get rid of first and last 5 seconds (to remove ground and transient effects)
Experiment2.u1 = Experiment2.u1(Experiment2.t>5 &...
    Experiment2.t < Experiment2.t(end)-5);
Experiment2.y1 = Experiment2.y1(Experiment2.t>5 &...
    Experiment2.t < Experiment2.t(end)-5);

thrust_data2 = iddata(Experiment2.y1',Experiment2.u1',Experiment2.Ts,...
    'ExperimentName', 'FireFlySysID_2', 'InputName','thrust_{cmd}',...
    'OutputName','thrust', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');

thrust_data2 = detrend(thrust_data2);   


%%
% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3


%Assume 1st  order system  
np = 2;
nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
thrust_estimated_tf1 = tfest(thrust_data1,np, nz);

[~, fit1, ~] = compare(thrust_data2, thrust_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
thrust_estimated_tf2 = tfest(thrust_data2,np, nz);

[~, fit2, ~] = compare(thrust_data1, thrust_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    thrust_estimated_tf = thrust_estimated_tf1;
    disp('The thrust model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(thrust_data2, thrust_estimated_tf1);
    disp(strcat('The thrust model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    thrust_estimated_tf = thrust_estimated_tf2;
    disp('The thrust model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(thrust_data1, thrust_estimated_tf2);
    disp(strcat('The thrust model fits the validation data with **',...
        num2str(fit2), '** %'));
end


%%% Estimated Transfer Functions

disp('Thrust estimated transfer function is: ');
tf(thrust_estimated_tf)
thrust_params=getpvec(thrust_estimated_tf);

if(np==1)
    thrust_gain=thrust_params(1)/thrust_params(2);
    thrust_tau=1/thrust_params(2);
    fprintf('thrust gain=%.3f, tau=%.3f\n',thrust_gain,thrust_tau);
elseif(np==2)
    thrust_params=getpvec(thrust_estimated_tf);
    thrust_omega=sqrt(thrust_params(3));
    thrust_gain=thrust_params(1)/thrust_params(3);
    thrust_damping=thrust_params(2)/(2*thrust_omega);
    fprintf('thrust omega=%.3f, gain=%.3f damping=%.3f\n',thrust_omega,thrust_gain,thrust_damping);
end




figure('Name','System analysis (thrust)');
subplot(311);
bode(thrust_estimated_tf);

title('thrust bode plot');

subplot(312);
rlocusplot(thrust_estimated_tf); grid;
title('thrust RootLucas plot');

subplot(313);
step(thrust_estimated_tf); grid;
title('thrust step response plot');