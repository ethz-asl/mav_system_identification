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

path(path, '../read_bags');

path(path, '../helper_functions');

% two experiments are needed to validate the identification
bagfile_exp1 =  'Experiment1/2014-11-27-15-03-25.bag';
bagfile_exp2 =  'Experiment2/2014-11-27-15-05-23.bag';

topic_imu = '/fcu/imu';
topic_motor_speed = '/fcu/motor_speed';
topic_rcdata = '/fcu/rcdata';
topic_vicon = '/vicon/auk/auk';

bag1 = ros.Bag(bagfile_exp1);
bag2 = ros.Bag(bagfile_exp2);
%%
% *First experriment info:*
bag1.info

%%
% *Second experiment info:*
bag2.info


%% 
Experiment1.IMU = readImu(bag1, topic_imu);
Experiment2.IMU = readImu(bag2, topic_imu);
Experiment1.MotorSpeed = readMotorSpeed(bag1, topic_motor_speed);
Experiment2.MotorSpeed = readMotorSpeed(bag2, topic_motor_speed);
Experiment1.Vicon = readTransformStamped(bag1, topic_vicon);
Experiment2.Vicon = readTransformStamped(bag2, topic_vicon);
Experiment1.RCData = readRCData(bag1, topic_rcdata);
Experiment2.RCData = readRCData(bag2, topic_rcdata);


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


Experiment1.rpy(2,:) = -Experiment1.rpy(2,:);
Experiment2.rpy(2,:) = -Experiment2.rpy(2,:);


%time from 0
Experiment1.Vicon.t = Experiment1.Vicon.t - Experiment1.Vicon.t(1);
Experiment1.RCData.t = Experiment1.RCData.t - Experiment1.RCData.t(1);
Experiment1.IMU.t = Experiment1.IMU.t - Experiment1.IMU.t(1);

Experiment2.Vicon.t = Experiment2.Vicon.t - Experiment2.Vicon.t(1);
Experiment2.RCData.t = Experiment2.RCData.t - Experiment2.RCData.t(1);
Experiment2.IMU.t = Experiment2.IMU.t - Experiment2.IMU.t(1);


 %commands
% This conversion is not fixed, change it accordingly
k_pitch_roll = (51.2*pi/180)/2047;
k_thrust = 100/4095;

Experiment1.roll_cmd    = (Experiment1.RCData.channel(2,:)-2047)...
    *k_pitch_roll;
Experiment1.pitch_cmd   = (Experiment1.RCData.channel(1,:)-2047)...
    *k_pitch_roll;
Experiment1.thrust_cmd  = Experiment1.RCData.channel(3,:)...
    *k_thrust;

Experiment2.roll_cmd    = (Experiment2.RCData.channel(2,:)-2047)...
    *k_pitch_roll;
Experiment2.pitch_cmd   = (Experiment2.RCData.channel(1,:)-2047)...
    *k_pitch_roll;
Experiment2.thrust_cmd  = Experiment2.RCData.channel(3,:)*k_thrust;


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
% *Plot attitude from experiment 1*
figure(2);
title('Experiment 1 Data');
subplot(2,1,1);
plot(Experiment1.Vicon.t, Experiment1.rpy(1,:)*180/pi, ...
    Experiment1.RCData.t, Experiment1.roll_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('roll [deg]');
title('roll from vicon');

subplot(2,1,2);
plot(Experiment1.Vicon.t, Experiment1.rpy(2,:)*180/pi, ...
    Experiment1.RCData.t, Experiment1.pitch_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from vicon');




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
%%
% *Plot attitude from experiment 2*
title('Experiment 2 Data');
subplot(2,1,1);
plot(Experiment2.Vicon.t, Experiment2.rpy(1,:)*180/pi, ...
    Experiment2.RCData.t, Experiment2.roll_cmd*180/pi,...
    'g--', 'linewidth', 2);

legend('y','y_{ref}');
xlabel('time');
ylabel('roll [deg]');
title('roll from vicon');

subplot(2,1,2);
plot(Experiment2.Vicon.t, Experiment2.rpy(2,:)*180/pi, ...
    Experiment2.RCData.t, Experiment2.pitch_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from vicon');


%% Identification of roll system
Experiment1.t = (Experiment1.Vicon.t(1:end-1) + Experiment1.RCData.t)/2;
Experiment1.u1 = Experiment1.roll_cmd;
Experiment1.y1 = Experiment1.rpy(1,:);
Experiment1.Ts = mean(diff(Experiment1.Vicon.t));

%%
% *get rid of first and last 10 seconds (to remove ground and transient effects)*
Experiment1.u1 = Experiment1.u1(Experiment1.t>10 & ...
    Experiment1.t < Experiment1.t(end)-10);
Experiment1.y1 = Experiment1.y1(Experiment1.t>10 &...
    Experiment1.t < Experiment1.t(end)-10);
%Experiment1.t = Experiment1.t(Experiment1.t>10 & Experiment1.t < Experiment1.t(end)-10);

roll_data1 = iddata(Experiment1.y1',Experiment1.u1',Experiment1.Ts, ...
    'ExperimentName', 'FireFlySysID_1', 'InputName','roll_{cmd}', ...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

roll_data1 = detrend(roll_data1);



Experiment2.t = (Experiment2.Vicon.t(1:end-1) + Experiment2.RCData.t)/2;
Experiment2.u1 = Experiment2.roll_cmd;
Experiment2.y1 = Experiment2.rpy(1,:);
Experiment2.Ts = mean(diff(Experiment2.Vicon.t));


%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment2.u1 = Experiment2.u1(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-10);
Experiment2.y1 = Experiment2.y1(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-10);

roll_data2 = iddata(Experiment2.y1',Experiment2.u1',Experiment2.Ts,...
    'ExperimentName', 'FireFlySysID_2', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');

roll_data2 = detrend(roll_data2);   


%%
% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3


%Assume 2nd order system  
np = 2;
nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
roll_estimated_tf1 = tfest(roll_data1,np, nz);

[~, fit1, ~] = compare(roll_data2, roll_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
roll_estimated_tf2 = tfest(roll_data2,np, nz);

[~, fit2, ~] = compare(roll_data1, roll_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    roll_estimated_tf = roll_estimated_tf1;
    disp('The roll model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(roll_data2, roll_estimated_tf1);
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    roll_estimated_tf = roll_estimated_tf2;
    disp('The roll model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(roll_data1, roll_estimated_tf2);
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit2), '** %'));
end


%% Identification of Pitch System
Experiment1.u2 = Experiment1.pitch_cmd;
Experiment1.y2 = Experiment1.rpy(2,:);

%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment1.u2 = Experiment1.u2(Experiment1.t>10 &...
    Experiment1.t < Experiment1.t(end)-10);
Experiment1.y2 = Experiment1.y2(Experiment1.t>10 &...
    Experiment1.t < Experiment1.t(end)-10);
Experiment1.t = Experiment1.t(Experiment1.t>10 &...
    Experiment1.t < Experiment1.t(end)-10);

pitch_data1 = iddata(Experiment1.y2',Experiment1.u2',Experiment1.Ts,...
    'ExperimentName', 'FireFlySysID_1', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');

%remove any trend in the data
pitch_data1 = detrend(pitch_data1);



Experiment2.u2 = Experiment2.pitch_cmd;
Experiment2.y2 = Experiment2.rpy(2,:);


%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment2.u2 = Experiment2.u2(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-10);
Experiment2.y2 = Experiment2.y2(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-10);
Experiment2.t = Experiment2.t(Experiment2.t>10 &...
    Experiment2.t < Experiment2.t(end)-10);

pitch_data2 = iddata(Experiment2.y2',Experiment2.u2',Experiment2.Ts, ...
    'ExperimentName', 'FireFlySysID_2', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');
pitch_data2 = detrend(pitch_data2);   


%%
% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3
  
%Assume 2nd order system
np = 2;
nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
pitch_estimated_tf1 = tfest(pitch_data1,np, nz);

[~, fit1, ~] = compare(pitch_data2, pitch_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
pitch_estimated_tf2 = tfest(pitch_data2,np, nz);

[~, fit2, ~] = compare(pitch_data1, pitch_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    pitch_estimated_tf = pitch_estimated_tf1;
    disp('The pitch model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(pitch_data2, pitch_estimated_tf1);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    pitch_estimated_tf = pitch_estimated_tf2;
    disp('The pitch model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(pitch_data1, pitch_estimated_tf2);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit2), '** %'));
end



%% Estimate the Whole System as 2-input 2-output MIMO System
% *The purpose here is to see of there is coupling*

Experiment2.Ts = Experiment1.Ts;    
Data1 = iddata([Experiment1.y1', Experiment1.y2'], ...
    [Experiment1.u1', Experiment1.u2'], Experiment1.Ts, ...
    'ExperimentName', 'FireFlySysID_1', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'},...
    'TimeUnit','Second');


                          
Data2 = iddata([Experiment2.y1', Experiment2.y2'], ...
    [Experiment2.u1', Experiment2.u2'], Experiment2.Ts, ...
    'ExperimentName', 'FireFlySysID_2', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'}, ...
    'TimeUnit','Second');


MergedData = merge(Data1, Data2);

np = 2;
nz = 0;
Full_estimated_tf = tfest(MergedData, np,nz);

figure;
bodemag(Full_estimated_tf);


%%% Estimated Transfer Functions

disp('Roll estimated transfer function is: ');
tf(roll_estimated_tf)
figure;
bode(roll_estimated_tf); grid;
title('Roll bode plot');

disp('Pitch estimated transfer function is: ');
tf(pitch_estimated_tf)
figure;
bode(pitch_estimated_tf); grid;
title('Pitch bode plot');

% %% Compute total acceleration
% 
% N1 = length(Experiment1.Vicon.t);
% Experiment1.Acc = zeros(3,N1);
% N2 = length(Experiment2.Vicon.t);
% Experiment2.Acc = zeros(3,N2);
% 
% 
% for i=3:N1-2
%     Experiment1.Acc(:,i) = (Experiment1.Vicon.p(:,i+2)...
%         - 2*Experiment1.Vicon.p(:,i) ...
%         + Experiment1.Vicon.p(:,i-2))/(2*Experiment1.Ts^2);
% end
% for i=3:N2-2
%     Experiment2.Acc(:,i) = (Experiment2.Vicon.p(:,i+2)...
%         - 2*Experiment2.Vicon.p(:,i) ...
%         + Experiment2.Vicon.p(:,i-2))/(2*Experiment2.Ts^2);
% end
% 
% T1x = diag(sqrt(Experiment1.Acc(1,:)'*Experiment1.Acc(1,:)));
% T1y = diag(sqrt(Experiment1.Acc(2,:)'*Experiment1.Acc(2,:)));
% T1z = diag(sqrt(Experiment1.Acc(3,:)'*Experiment1.Acc(3,:))) + 9.81;
% 
% T1_mod = sqrt(T1x.^2 + T1y.^2 + T1z.^2);    
% 
% T2x = diag(sqrt(Experiment2.Acc(1,:)'*Experiment2.Acc(1,:)));
% T2y = diag(sqrt(Experiment2.Acc(2,:)'*Experiment2.Acc(2,:)));
% T2z = diag(sqrt(Experiment2.Acc(3,:)'*Experiment2.Acc(3,:))) + 9.81;
% 
% T2_mod = sqrt(T2x.^2 + T2y.^2 + T2z.^2);
% 
% sqr_motor_speed_sum1 = sum((Experiment1.MotorSpeed.motor_speed*2*pi).^2,1);
% sqr_motor_speed_sum2 = sum((Experiment2.MotorSpeed.motor_speed*2*pi).^2,1);









 