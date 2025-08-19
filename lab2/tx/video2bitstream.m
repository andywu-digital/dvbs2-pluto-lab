function [bitstream, width, height] = video2bitstream(video, scale)

bitstream = [];
frame = readFrame(video);

for i = 1:149
    frame = readFrame(video);
    if(mod(i, 10) == 0)
        s_frame = imresize(frame, scale);
        bitstream_s = pic2bitstream(s_frame);
        bitstream = [bitstream; bitstream_s];
    end
end

width = size(s_frame, 2);
height = size(s_frame, 1);

