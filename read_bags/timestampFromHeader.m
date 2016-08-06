function timestamp = timestampFromHeader(header)

timestamp = double(header.stamp.sec) + double(header.stamp.nsec)*1e-9;