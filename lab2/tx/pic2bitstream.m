function [bitstream] = pic2bitstream(img)

R = img(:,:,1,:);
G = img(:,:,2,:);
B = img(:,:,3,:);

R = R(:);
G = G(:);
B = B(:);

rBits = dec2bin(R, 8);
gBits = dec2bin(G, 8);
bBits = dec2bin(B, 8);

bitstream = reshape([rBits gBits bBits].' - '0', [], 1);



end