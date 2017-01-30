function pose = readDJI_SimulPose(bag, topic_IMU, topic_position)
	position_raw = bag.readAll(topic_position);
	imu_raw = bag.readAll(topic_IMU);
    
    %Match the length of IMU and position data.
    cnt_del_imu=0;
    cnt_del_position=0;
    while(length(imu_raw)~=length(position_raw))
        if length(imu_raw)>length(position_raw)
            imu_raw(end)=[];
            cnt_del_imu=cnt_del_imu+1;
        else
            position_raw{end}=[];
            cnt_del_position=cnt_del_position+1;
        end
    end
    if cnt_del_imu>0
        fprintf('%d deleted from IMU data\n',cnt_del_imu);
    elseif cnt_del_position>0
        fprintf('%d deleted from position data\n',cnt_del_position);
    end
    
    sz = length(position_raw);

	pose.t = zeros(1, sz);
	pose.i = zeros(1, sz);
	pose.p = zeros(3, sz);
	pose.q = zeros(4, sz);

	for i=1:sz
	   pose.t(i) = timestampFromHeader(position_raw{i}.header);
	   pose.i(i) = position_raw{i}.header.seq;
	   pose.p(1,i) = position_raw{i}.x;
	   pose.p(2,i) = position_raw{i}.y;
	   pose.p(3,i) = position_raw{i}.z;
	   pose.q(:,i) = imu_raw{i}.orientation;
	end