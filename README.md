# mav_system_identification
Matlab scripts to perform system identification for DJI 100

In order to run this script, m100_sysid.m, you need matlab_rosbag package
https://github.com/bcharrow/matlab_rosbag (source)
https://github.com/bcharrow/matlab_rosbag/releases (binary)
 
In case you face the follosing linking error
matlab_rosbag-0.5.0-mac64/rosbag_wrapper.mexmaci64,
6): Symbol not found: __ZTISt16invalid_argument
try this re-compiled binary
https://cmu.app.box.com/s/9hs153nwa19uqvzboglkz7y84r6jzzxg    or
https://drive.google.com/open?id=10L4LHh1icdken60UTwKTRoUvLG3fO-1m
Tested platform: Mac EI Capitan 10.11.6 with MATLAB R2016a
