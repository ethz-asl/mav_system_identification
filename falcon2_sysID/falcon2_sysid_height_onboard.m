clear all
path(path, '../read_bags');
path(path, '../helper_functions');

% two experiments are needed to validate the identification
bagfile_exp2 =  '../bags/falcon2_exp02_again_again.bag';

topic_imu = '/falcon2/dji_sdk/imu';
topic_rcdata = '/falcon2/dji_sdk/vc_cmd';
topic_vel = '/falcon2/dji_sdk/velocity';

bag2 = ros.Bag(bagfile_exp2);

%%
% *Second experiment info:*
bag2.info


%% Prepare datasets
Experiment2.IMU = readImu(bag2, topic_imu);
Experiment2.RCData = readCommnadRollPitchYawRateThrust(bag2, topic_rcdata);
Experiment2.Vel = readDJI_Velocity(bag2,topic_vel);

% Write the quaternions, w, x,y,z order!!
Experiment2.IMU.q = [Experiment2.IMU.q(4,:); Experiment2.IMU.q(1,:); ...
    Experiment2.IMU.q(2,:); Experiment2.IMU.q(3,:)];


Experiment2.rpy_imu = quat2rpy(Experiment2.IMU.q);

Experiment2.RCData.t = Experiment2.RCData.t - Experiment2.RCData.t(1);
Experiment2.IMU.t = Experiment2.IMU.t - Experiment2.IMU.t(1);
Experiment2.Vel.t = Experiment2.Vel.t - Experiment2.Vel.t(1);

%========================================================
%IMU, RCDATA interpolation with respect to onboard vz
%Exp2
%========================================================

imu_q1_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(1,:),Experiment2.Vel.t,'spline');
imu_q2_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(2,:),Experiment2.Vel.t,'spline');
imu_q3_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(3,:),Experiment2.Vel.t,'spline');
imu_q4_intp=interp1(Experiment2.IMU.t,Experiment2.IMU.q(4,:),Experiment2.Vel.t,'spline');

cmd_ch1_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.roll,Experiment2.Vel.t,'spline');
cmd_ch2_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.pitch,Experiment2.Vel.t,'spline');
cmd_ch3_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.yaw_rate,Experiment2.Vel.t,'spline');
cmd_ch4_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.thrust(3,:),Experiment2.Vel.t,'spline');

Experiment2.IMU.q=zeros(4,size(imu_q1_intp,2));
Experiment2.IMU.q=[imu_q1_intp;imu_q2_intp;imu_q3_intp;imu_q4_intp];
Experiment2.IMU.t=zeros(1,size(imu_q1_intp,2));
Experiment2.IMU.t=Experiment2.Vel.t;

Experiment2.RCData.channel=zeros(4,size(cmd_ch1_intp,2));
Experiment2.RCData.channel=[cmd_ch1_intp;cmd_ch2_intp;cmd_ch3_intp;cmd_ch4_intp]; %roll, pitch, yaw rate, vertical vel
Experiment2.RCData.t=zeros(1,size(cmd_ch1_intp,2));
Experiment2.RCData.t=Experiment2.Vel.t;
Experiment2.RCData=rmfield(Experiment2.RCData,'i');
Experiment2.RCData=rmfield(Experiment2.RCData,'roll');
Experiment2.RCData=rmfield(Experiment2.RCData,'pitch');
Experiment2.RCData=rmfield(Experiment2.RCData,'yaw_rate');
Experiment2.RCData=rmfield(Experiment2.RCData,'thrust');


% For DJI M100 platform.
%Please have a look "DJI_M100_regression for more detail.
k_pitch = 0.000844; 
k_roll  = 0.000865;
k_thrust = 0.0029;

%DJI vc channel, 1=pitch, 2=roll, 3=thrust, 4=yaw_rate.

Experiment2.roll_cmd    = (Experiment2.RCData.channel(2,:)-1024)...
    *k_pitch;
Experiment2.pitch_cmd   = (Experiment2.RCData.channel(1,:)-1024)...
    *k_roll;
Experiment2.thrust_cmd  = (Experiment2.RCData.channel(4,:)-1024)*k_thrust;

close all;
figure;
title('Experiment 2 Data');
subplot(1,1,1);
plot(Experiment2.Vel.t, Experiment2.Vel.vz, ...
    Experiment2.RCData.t, Experiment2.thrust_cmd, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('m/s');
title('dz from onboard');


%% Identification of vertical velocity system
%% The length of data may vary.
Experiment2.t = (Experiment2.Vel.t + Experiment2.RCData.t)/2;
Experiment2.u1 = Experiment2.thrust_cmd;
Experiment2.y1 = Experiment2.Vel.vz;
Experiment2.Ts = mean(diff(Experiment2.Vel.t));

%%
% *get rid of first and last 10 seconds (to remove ground and transient effects)*
Experiment2.u1 = Experiment2.u1(Experiment2.t>10 & ...
    Experiment2.t < Experiment2.t(end)-49);
Experiment2.y1 = double(Experiment2.y1(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-49));

figure;
plot(Experiment2.u1,'b');
hold on;
plot(Experiment2.y1,'r');
legend('input_{cmd dz}','output_{onboard dz}');
xlabel('Samples');
ylabel('m/s');
grid on;

thrust_data2 = iddata(Experiment2.y1',Experiment2.u1',Experiment2.Ts, ...
    'ExperimentName', 'FireFlySysID_1', 'InputName','thrust_{cmd}', ...
    'OutputName','thrust', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

thrust_data2 = detrend(thrust_data2);

%Assume 1st  order system  
np = 1;
nz = 0;

%Generate model using Experiment2 and validate the model with Experiment2
thrust_estimated_tf = tfest(thrust_data2,np, nz);
[~, fit, ~] = compare(thrust_data2, thrust_estimated_tf);
%%% Estimated Transfer Functions
figure;
compare(thrust_data2, thrust_estimated_tf);
grid on;
    
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