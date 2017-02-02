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
load data/ExperimentYaw.mat
yaw_vc_cmd = Experiment1.RCData.channel(4,:);
yaw_vc_cmd = -(yaw_vc_cmd-1024);


% pitch_vc_cmd = Experiment1.RCData.channel(1,:);
% pitch_vc_cmd = pitch_vc_cmd-1024;

rc_t=Experiment1.RCData.t;

vicon_yaw=Experiment1.rpy(3,:); %in rad!!!
vicon_t=Experiment1.Vicon.t; % in second.
ts = mean(diff(vicon_t)); % in second.
dvicon_yaw=diff(vicon_yaw)/ts;
dvicon_yaw=[dvicon_yaw dvicon_yaw(end)];

figure('Name','Before regression Yaw');
plot(rc_t,yaw_vc_cmd,'b');
hold on;
plot(vicon_t,dvicon_yaw,'r');
xlabel('Time(s)');
ylabel('Rad/s');
legend('dyaw_{cmd}','Vicon dyaw mea');
grid on;

[yaw,lag_yaw]=xcorr(yaw_vc_cmd,dvicon_yaw);

[~,I_yaw]=max(yaw);
time_sync_yaw=lag_yaw(I_yaw);

rc_t = rc_t-time_sync_yaw*ts;

%========================
% Selecting sample range
%========================

start_t=10;
end_t=55;

b_idx=min(find(abs(rc_t - start_t)< ts));
e_idx=min(find(abs(rc_t - end_t)< ts));
samp_range_u=b_idx:e_idx-1;

b_idx=min(find(abs(vicon_t - start_t)< ts));
e_idx=min(find(abs(vicon_t - end_t)< ts));
samp_range_y=b_idx:e_idx;

u_yaw=detrend(yaw_vc_cmd(samp_range_u));
% u_pitch=pitch_vc_cmd(samp_range);
u_t=rc_t(samp_range_u);
y_dyaw=detrend(dvicon_yaw(samp_range_y));
% y_pitch=vicon_pitch(samp_range);
y_t=vicon_t(samp_range_y);
y_t(end)=[];
y_dyaw(end)=[];


%==================================
% nonlinear optimisation for yaw
%==================================
f=@(x)sum((y_dyaw - x*u_yaw).^2);
options = optimset('Display','Iter','MaxIter',2000,...
                   'MaxFunEvals',20000,...
                   'TolFun',1e-99,...
                   'TolX',1e-99);
th=0.1;
[x_yaw,fval,exitflag] = fminsearch(f,th,options);
fprintf('x_yaw=%f\n',x_yaw);
%x_yaw=-0.00323;
figure('Name','After regression yaw');
plot(u_t,u_yaw*x_yaw,'r');
hold on;
plot(y_t,y_dyaw,'b');

xlabel('Time(s)');
ylabel('Rad/s');
strLeg_Yaw=sprintf('yaw_{cmd}*%f, range from %.2fs ~ %.2fs',x_yaw,rc_t(b_idx),rc_t(e_idx));
legend(strLeg_Yaw,'Vicon diff(yaw) mea');
grid on;

