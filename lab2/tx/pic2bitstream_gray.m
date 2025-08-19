function [bitstream] = pic2bitstream_gray(img)

img_vector = img(:); 
bit_matrix = dec2bin(img_vector, 8);
bitstream = reshape(bit_matrix.' - '0', [], 1);

end