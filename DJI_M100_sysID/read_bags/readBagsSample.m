bagfile = '~/bags/20140403_leo_hummy/2014-04-03-17-09-48.bag';
topic_transform = '/vicon/hummy_2/hummy_2';
topic_control = '/hummy/fcu/control_new';

% open bagfile
bag = ros.Bag(bagfile);

% print bag info
bag.info

% read transform stamped into struct
vicon = readTransformStamped(bag, topic_transform);

% read control topic
control = readControl(bag, topic_control);
