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
load data/ExperimentHeight_falcon2.mat;
height_vc_cmd = Experiment2.RCData.channel(4,:);  %stick velocity command.
height_vc_cmd = height_vc_cmd-1024;


% pitch_vc_cmd = Experiment1.RCData.channel(1,:);
% pitch_vc_cmd = pitch_vc_cmd-1024;

rc_t=Experiment2.RCData.t;

vicon_z=Experiment2.Vicon.p(3,:); %in metre
vicon_t=Experiment2.Vicon.t; % in second.
ts = mean(diff(vicon_t)); % in second.
dvicon_z=diff(vicon_z)/ts;
dvicon_z=[dvicon_z dvicon_z(end)];

%========================
% Selecting sample range
%========================

start_t=32;
end_t=52;
b_idx=min(find(abs(rc_t - start_t)< ts));
e_idx=min(find(abs(rc_t - end_t)< ts));
samp_range_u=b_idx:e_idx;

b_idx=min(find(abs(vicon_t - start_t)< ts));
e_idx=min(find(abs(vicon_t - end_t)< ts));
samp_range_y=b_idx:e_idx;

u_dz=detrend(height_vc_cmd(samp_range_u));
u_t=rc_t(samp_range_u);
y_dz=detrend(dvicon_z(samp_range_y));
y_t=vicon_t(samp_range_y);


figure('Name','Before regression dz');
plot(u_t,u_dz,'b');
hold on;
plot(y_t,y_dz,'r');
xlabel('Time(s)');
ylabel('m/s');
legend('height_{cmd}','Vicon dz mea');
grid on;

[z,lag_z]=xcorr(height_vc_cmd,dvicon_z);
z=medfilt1(z,46);
[~,I_z]=max(z);
time_sync_z=lag_z(I_z);
rc_t = rc_t-time_sync_z*ts;
u_t=u_t-time_sync_z*ts;

y_dz=y_dz(-time_sync_z:end);
u_dz=u_dz(1:end+time_sync_z+1);

%==================================
% nonlinear optimisation for dz
%==================================
f=@(x)sum((y_dz - x*u_dz).^2);
options = optimset('Display','Iter','MaxIter',2000,...
                   'MaxFunEvals',20000,...
                   'TolFun',1e-99,...
                   'TolX',1e-99);
th=0.1;
[x_dz,fval,exitflag] = fminsearch(f,th,options);
x_dz

fprintf('x_dz=%f\n',x_dz);
figure('Name','After regression dz');
plot(u_dz*x_dz,'r','linewidth',2);
hold on;
plot(y_dz,'b');

xlabel('Time(s)');
ylabel('m/s');
strLeg_dz=sprintf('dz_{cmd}*%f, range from %.2fs ~ %.2fs',x_dz,rc_t(b_idx),rc_t(e_idx));
legend(strLeg_dz,'Vicon dz mea');
grid on;

