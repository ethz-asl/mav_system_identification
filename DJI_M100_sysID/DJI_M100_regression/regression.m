%Copyright (C) 2016, by Inkyu Sa, enddl22@gmail.com

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
load regression.mat
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
legend('roll_{cmd}','Vicon roll mea');
grid on;


figure('Name','Before regression Pitch');
plot(rc_t,pitch_vc_cmd,'b');
hold on;
plot(vicon_t,rad2deg(vicon_pitch),'r');
xlabel('Time(s)');
ylabel('Deg');
legend('pitch_{cmd}','Vicon pitch_ mea');
grid on;


%========================
% Selecting sample range
%========================

start_t=40;
end_t=120;
b_idx=min(find(abs(rc_t - start_t)< ts));
e_idx=min(find(abs(rc_t - end_t)< ts));
samp_range=b_idx:e_idx;

u_roll=roll_vc_cmd(samp_range);
u_pitch=pitch_vc_cmd(samp_range);
u_t=rc_t(samp_range);
y_roll=vicon_roll(samp_range);
y_pitch=vicon_pitch(samp_range);
y_t=vicon_t(samp_range);

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
legend(strLeg_Roll,'Vicon roll mea');
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
legend(strLeg_Pitch,'Vicon pitch mea');
grid on;

