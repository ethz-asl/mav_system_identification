

dirbase = '/scratch/bos057/ethz/Vicon/PTAM_ASLIMU_CrossbowIMU'

fname_vicon = fullfile(dirbase,'Vicon','PTAM_220709_1602.txt');
fname_csbw = fullfile(dirbase,'Crossbow','data_samples_crossbow2009-07-22-140317.txt');
fname_ptam = fullfile(dirbase,'PTAM','CamFrames2009-07-22-140324','poses.txt');

% fix up missing sf
sf = 1.5*200/180;
d_csbw(:,6:8) = d_csbw(:,6:8)*sf;

d_csbw = load(fname_csbw);
t_csbw = d_csbw(:,1) - d_csbw(1,1);
t2_csbw = unwrap(d_csbw(:,2)*(2*pi/(2^16)),pi*1.4)/(2*pi/(2^16)) * 0.79e-6;

p = polyfit(1:length(t_csbw),t_csbw',1)
dt_csbw = p(1);
ts_csbw = t2_csbw * t_csbw(end)/t2_csbw(end); %(0:length(t_csbw)-1)'*dt_csbw;
r_csbw = d_csbw(:,6:8);

%estimate rate bias
N = 31;
sr_csbw = sqrt(filter(ones(N,1)/N,1,(sum(filter([1 zeros(1,N-1) -1],1,r_csbw).^2,2))));

g = find(sr_csbw < 0.07);
r_bias = mean(r_csbw(g,:));

r_csbw = r_csbw - r_bias(ones(1,length(r_csbw)),:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

d_vicon = load(fname_vicon);
t_vicon = d_vicon(:,1)-d_vicon(1,1);
p = polyfit(1:length(t_vicon),t_vicon',1)
dt_vicon = p(1);
ts_vicon = (0:length(t_vicon)-1)'*dt_vicon;
q_vicon = rpy2quat(diag([1 1 1])*d_vicon(:,6:8)')';

%interpolate holes in vicon data
g = find(any(d_vicon(:,3:5),2));
b = find(all(d_vicon(:,3:5)==0,2));
d_vicon(b,3:8) = interp1(g,d_vicon(g,3:8),b,'linear',0);
q_vicon(b,:) = renorm(interp1(g,unwrap_quat(q_vicon(g,:)')',b,'linear',0)')';
q_vicon = unwrap_quat(q_vicon')';


r_vicon = quat2rotvel(mul_quat(diag([1 -1 -1 -1])*q_vicon([1:end],:)',q_vicon([2:end 1],:)'))'/dt_vicon;

z1 = sqrt(sum(r_csbw'.^2))';
z2 = sqrt(sum(r_vicon'.^2))';
x = time_reg(ts_vicon,z2,ts_csbw,z1);
delta_t = x(1);
delta_dt = x(2);

% interpolate vicon data and csbw data at 100Hz
dt_s = 1/100;
ts100 = ts_csbw(1):dt_s:ts_csbw(end);
d_vicon100 = interp1(ts_vicon*(1+delta_dt)+delta_t,d_vicon,ts100,'linear',0);
q_vicon100 = unwrap_quat(renorm(interp1(ts_vicon*(1+delta_dt)+delta_t,q_vicon,ts100,'linear','extrap')'))';


r_vicon100 = quat2rotvel(mul_quat(diag([1 -1 -1 -1])*q_vicon100([end 1:end-1],:)',q_vicon100([2:end 1],:)'))'/(2*dt_s);

d_csbw100 = interp1(ts_csbw,d_csbw,ts100,'pchip');
r_csbw100 = d_csbw100(:,6:8);


plot([r_csbw100 r_vicon100])

%zoom in to valid region
a = round(axis); g = (a(1):a(2))'; size(g)


A = (r_csbw100(g,:)\r_vicon100(g,:))'
[u,s,v] = svd(A); diag(s)
RviconFromCsbw = u*v'

plot([(RviconFromCsbw*r_csbw100')' r_vicon100])


plot3mc([r_csbw(g,:) g]'); axis equal




%%%%%%%%%%%%%%%%%%%%%

d_ptam = load(fname_ptam);
t_ptam = d_csbw(:,1) - d_csbw(1,1);


%% convert data and time
pos_ptam = d_ptam(:,5:7);
Rs_ptam = reshape(d_ptam(:,8:16),[],3,3);
Rs_ptam = permute(Rs_ptam,[3,2,1]);
q_ptam = rot2quat(Rs_ptam);
tbuff = num2str(d_ptam(:,4));
t_ptam = ((str2num(tbuff(:,1:end-7)).*60.+str2num(tbuff(:,end-6:end-5))).*60.+str2num(tbuff(:,end-4:end-3)))+str2num(tbuff(:,end-2:end))/1000; %#ok<ST2NM>
t_ptam = t_ptam-t_ptam(1);



%%%%%%%%%%%%%555



% project gravity into body coordinates
gb_vicon = quat_rotate(diag([1 -1 -1 -1])*q_vicon_csbw',[0 0 -9.8]')';



% find periods of stillness
N = 31;
sr = sqrt(filter(ones(N,1)/N,1,(sum(filter([1 zeros(1,N-1) -1],1,q_vicon_csbw).^2,2))));
g = find(sr < 12e-4);

dg = [find(diff(g)>1)+1];

gi = g(dg(find(diff(dg)>1000)))+500;



[tvi,s1,s2,T] = absolute_orientation(gb_vicon(gi,:)',-d(gi,1:3)')

acc_v = transform_points(tvi,d_csbw(:,3:5)')' * s2/s1;

A = [d_imu(gi,1:3) ones(length(gi),1)]\gb_vicon(gi,:)

acc_v = [d_imu(:,1:3) ones(length(d_imu),1)]*A;
