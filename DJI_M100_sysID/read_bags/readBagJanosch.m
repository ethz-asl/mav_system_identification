bagfile = '/home/burrimi/2014-06-20-18-42-04.bag';

topic_imu = '/firefly/imu';

bag = ros.Bag(bagfile);

bag.info


imu = readImu(bag, topic_imu);
