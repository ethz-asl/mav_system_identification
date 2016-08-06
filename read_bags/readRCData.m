function rcdata = readRCData( bag, topic )

rcdata_raw = bag.readAll(topic);
sz = length(rcdata_raw);

rcdata.t = zeros(1, sz);
rcdata.i = zeros(1,sz);
rcdata.channel = zeros(8, sz);

for i=1:sz
    rcdata.t(i) = timestampFromHeader(rcdata_raw{i}.header);
    rcdata.i(i) = rcdata_raw{i}.header.seq;
    rcdata.channel(:,i) = rcdata_raw{i}.channel;
end

