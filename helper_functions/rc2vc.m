function [ vcdata ] = rc2vc( rcdata )

% Code stealed from https://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.1/dji_sdk_lib/src/DJI_VirtualRC.cpp
%{
VirtualRCData VirtualRC::toVirtualRCData(RadioData &rData)
{
  VirtualRCData vd;
  vd.yaw = rData.yaw * 660 / 10000 + 1024;
  vd.throttle = rData.throttle * 660 / 10000 + 1024;
  vd.pitch = rData.pitch * 660 / 10000 + 1024;
  vd.roll = rData.roll * 660 / 10000 + 1024;
  vd.gear = (rData.gear == -4545) ? 1324 : 1684;
  vd.reserved = 1024;
  vd.mode = rData.mode * 660 / (-10000) + 1024;
  vd.Channel_07 = 1024;
  vd.Channel_08 = 1024;
  vd.Channel_09 = 1024;
  vd.Channel_10 = 1024;
  vd.Channel_11 = 1024;
  vd.Channel_12 = 1024;
  vd.Channel_13 = 1024;
  vd.Channel_14 = 1024;
  vd.Channel_15 = 1024;
  return vd;
}
%}

    sz=length(rcdata.i);
    vcdata.t = zeros(1, sz);
    vcdata.i = zeros(1,sz);

    vcdata.channel=zeros(16,sz);
    
    for i=1:sz
        vcdata.t(i) = rcdata.t(i);
        vcdata.i(i) = rcdata.i(i);
        vcdata.channel(1,i) = rcdata.channel(1,i)* 660 / 10000 + 1024;  %pitch
        vcdata.channel(2,i) = rcdata.channel(2,i)* 660 / 10000 + 1024;  %roll
        vcdata.channel(3,i) = rcdata.channel(3,i) * 660 / 10000 + 1024; %throttle
        vcdata.channel(4,i) = rcdata.channel(4,i) * 660 / 10000 + 1024; %yaw
    end
end

