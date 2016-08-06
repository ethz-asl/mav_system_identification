bagfile = '/home/burrimi/bags/20140403_leo_hummy/2014-04-03-17-09-48.bag';
topic_transform = '/vicon/hummy_2/hummy_2';
topic_control = '/hummy/fcu/control_new';

bag = ros.Bag(bagfile);

bag.info

vicon = readTransformStamped(bag, topic_transform);

control = readControl(bag, topic_control);

vicon.t=vicon.t(:,1100:2300);
vicon.i=vicon.i(:,1100:2300);
vicon.p=vicon.p(:,1100:2300);
vicon.q=vicon.q(:,1100:2300);

%% take derivative and smooth
dt=0.01;
pp=csaps(vicon.t,[diff(vicon.p(1,:),1) 0]/dt,0.9999);
v_x = fnval(pp,vicon.t);
pp=csaps(vicon.t,[diff(vicon.p(2,:),1) 0]/dt,0.9999);
v_y = fnval(pp,vicon.t);
pp=csaps(vicon.t,[diff(vicon.p(3,:),1) 0]/dt,0.9999);
v_z = fnval(pp,vicon.t);

%% get angles
% rpy_ref=quat2rpy(control.q)*180/pi;
rpy_meas=quat2rpy(vicon.q)*180/pi;

%% plot
close all
figure

subplot(3,2,1)
hold on;
plot(vicon.t,[vicon.p(1,:);vicon.p(2,:);vicon.p(3,:)],'--');
plot(control.t,[control.p(1,:);control.p(2,:);control.p(3,:)]);
title('pos')
grid on
legend('x','y','z')
%%
% figure
subplot(3,2,3)
hold on;


plot(vicon.t,[v_x;v_y;v_z],'--');
plot(control.t,[control.v(1,:);control.v(2,:);control.v(3,:)]);
title('vel')
grid on
legend('x','y','z')
%%
% figure
subplot(3,2,5)

hold on;

plot(vicon.t,[0 diff(v_x,1)/dt;0 diff(v_y,1)/dt;0 diff(v_z,1)/dt],'--');
plot(control.t,[control.a(1,:);control.a(2,:);control.a(3,:)]);
title('acc')
grid on
legend('x','y','z')

subplot(3,2,2)

hold on;

plot(vicon.t,[rpy_meas(1,:);rpy_meas(2,:);rpy_meas(3,:)],'--');
% plot(control.t,[control.a(1,:);control.a(2,:);control.a(3,:)]);
title('acc')
grid on
legend('x','y','z')
