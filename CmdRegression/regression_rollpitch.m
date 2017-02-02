%Copyright (C) 2017, by 
%Inkyu Sa, inkyu.sa@mavt.ethz.ch
%Mina Kamel, mina.kamel@mavt.ethz.ch

%This is free software: you can redistribute it and/or modify
%it under the terms of the GNU Lesser General Public License as published by
%the Free Software Foundation, either version 3 of the License, or
%(at your option) any later version.
 
%This software package is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU Lesser General Public License for more details.

%You should have received a copy of the GNU Leser General Public License.
%If not, see <http://www.gnu.org/licenses/>.

close all
clear all
load data/ExperimentPitchRoll.mat
roll_vc_cmd = Experiment1.RCData.channel(2,:);
roll_vc_cmd = roll_vc_cmd-1024;

pitch_vc_cmd = Experiment1.RCData.channel(1,:);
pitch_vc_cmd = pitch_vc_cmd-1024;

rc_t=Experiment1.RCData.t;

vicon_roll=Experiment1.rpy(1,:); %in rad!!!
vicon_pitch=Experiment1.rpy(2,:); %in rad!!!
vicon_t=Experiment1.Vicon.t; % in second.
ts = mean(diff(vicon_t)); % in second.

figure('Name','Before regression Roll');
plot(rc_t,roll_vc_cmd,'b');
hold on;
plot(vicon_t,rad2deg(vicon_roll),'r');
xlabel('Time(s)');
ylabel('Deg');
legend('roll_{cmd}','IMU roll mea');
grid on;


figure('Name','Before regression Pitch');
plot(rc_t,pitch_vc_cmd,'b');
hold on;
plot(vicon_t,rad2deg(vicon_pitch),'r');
xlabel('Time(s)');
ylabel('Deg');
legend('pitch_{cmd}','IMU pitch_ mea');
grid on;

[roll,lag_roll]=xcorr(roll_vc_cmd,vicon_roll);

[~,I_roll]=max(roll);
time_sync_roll=lag_roll(I_roll);

rc_t = rc_t-time_sync_roll*ts;


%========================
% Selecting sample range
%========================

start_t=15; % in Sec
end_t=55; % in Sec


b_idx=min(find(abs(rc_t - start_t)< ts));
e_idx=min(find(abs(rc_t - end_t)< ts));
samp_range_u=b_idx:e_idx;

b_idx=min(find(abs(vicon_t - start_t)< ts));
e_idx=min(find(abs(vicon_t - end_t)< ts));
samp_range_y=b_idx:e_idx;


u_roll=detrend(roll_vc_cmd(samp_range_u));
u_pitch=detrend(pitch_vc_cmd(samp_range_u));
u_t=rc_t(samp_range_u);
y_roll=detrend(vicon_roll(samp_range_y));
y_pitch=detrend(vicon_pitch(samp_range_y));
y_t=vicon_t(samp_range_y);

%==================================
% nonlinear optimisation for roll
%==================================
f=@(x)sum((y_roll - x*u_roll).^2);
options = optimset('Display','Iter','MaxIter',2000,...
                   'MaxFunEvals',20000,...
                   'TolFun',1e-99,...
                   'TolX',1e-99);
th=0.1;
[x_roll,fval,exitflag] = fminsearch(f,th,options);
fprintf('x_roll=%f\n',x_roll);

figure('Name','After regression roll');
plot(rc_t,rad2deg(roll_vc_cmd*x_roll),'b');
hold on;
plot(vicon_t,rad2deg(vicon_roll),'r');
xlabel('Time(s)');
ylabel('Deg');
strLeg_Roll=sprintf('roll_{cmd}*%f, range from %.2fs ~ %.2fs',x_roll,rc_t(b_idx),rc_t(e_idx));
legend(strLeg_Roll,'IMU roll mea');
grid on;


%==================================
% nonlinear optimisation for pitch
%==================================
f=@(x)sum((y_pitch - x*u_pitch).^2);
options = optimset('Display','Iter','MaxIter',2000,...
                   'MaxFunEvals',20000,...
                   'TolFun',1e-99,...
                   'TolX',1e-99);
th=0.1;
[x_pitch,fval,exitflag] = fminsearch(f,th,options);
fprintf('x_pitch=%f\n',x_pitch);

figure('Name','After regression Pitch');
plot(rc_t,rad2deg(pitch_vc_cmd*x_pitch),'b');
hold on;
plot(vicon_t,rad2deg(vicon_pitch),'r');
xlabel('Time(s)');
ylabel('Deg (Pitch)');
strLeg_Pitch=sprintf('pitch_{cmd}*%f, range from %.2fs ~ %.2fs',x_pitch,rc_t(b_idx),rc_t(e_idx));
legend(strLeg_Pitch,'IMU pitch mea');
grid on;

