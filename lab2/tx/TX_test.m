clc;
close all;
clear all;

%=========================================================%
% Load LDPCParityMatrices
%=========================================================%
if ~exist('dvbs2xLDPCParityMatrices.mat','file')
    if ~exist('s2xLDPCParityMatrices.zip','file')
        url = 'https://ssd.mathworks.com/supportfiles/spc/satcom/DVB/s2xLDPCParityMatrices.zip';
        websave('s2xLDPCParityMatrices.zip',url);
        unzip('s2xLDPCParityMatrices.zip');
    end
    addpath('s2xLDPCParityMatrices');
end

%=========================================================%
% Device Setup
%=========================================================%
txsim.DeviceName = "Pluto";
txsim.RadioCenterFrequency = 915e6;
txsim.RadioGain = 0;

%=========================================================%
% DVB-S2 Configuration Setup
%=========================================================%
wg = dvbs2WaveformGenerator;
wg.FECFrame = "short";
wg.MODCOD = 24;                               
wg.DFL = getDFL(wg.MODCOD,wg.FECFrame);       % Default DFL is Kbch-80
wg.SamplesPerSymbol = 1;
wg.StreamFormat = "GS";

%=========================================================%
% DVB-S2 Image Setup
%=========================================================%
%=============== image ===============%
% img = imread('imag/dog.jpg');
% weight = size(img, 2);
% height = size(img, 1);
% bitstream = pic2bitstream(img);
%============== video ===============%
video = VideoReader("video/20250717_2345_Pixel Pup Joy_simple_compose_01k0cgzma6e83s0z7kebf8j6qm.mp4");
[bitstream, weight, height] = video2bitstream(video, 0.05);

wg.UPL = wg.DFL;
wg.UPL = floor(wg.UPL/24)*24 -15;
numFrames = ceil(length(bitstream)/(wg.UPL-9));                                          

%=========================================================%
% Generate Input Data
%=========================================================%
rng("default")

if strcmpi(wg.StreamFormat,"TS")
    syncBits = [0 1 0 0 0 1 1 1]';                        % Sync byte for TS packet is 47 Hex
    pktLen = 1496;                                        % UP length without sync bits is 1496
else
    if wg.UPL
        syncBits = randi([0 1],8,1);
        pktLen = wg.UPL - 9;                              % UP length without sync byte
    end
end

% For GS continuous streams
if strcmpi(wg.StreamFormat,"GS") && wg.UPL == 0
    numBits = wg.DFL*numFrames;
    data = randi([0 1],numBits,1);
else % For TS and GS packetized streams
    numPkts = wg.MinNumPackets*numFrames; 
    %txRawPkts = randi([0 1],pktLen,numPkts);
    %txRawPkts = ones(pktLen,numPkts);
    pic_bitstream = [bitstream; zeros(pktLen*numPkts-length(bitstream), 1)];
    SOF = [reshape(dec2bin(weight, 12) - '0', [], 1); reshape(dec2bin(height, 12) - '0', [], 1); zeros(pktLen-24, 1)];
    txRawPkts = [SOF reshape(pic_bitstream, pktLen, numPkts)];

    txPkts = [repmat(syncBits,1,numPkts+1); [1 zeros(1, numPkts)]; txRawPkts];
    data = txPkts(:);
end

%=========================================================%
% Generate DVB-S2 Waveform
%=========================================================%
wg.HasPilots = true;
txOut = [wg(data); flushFilter(wg)];

%=========================================================%
% Configure SDR for Transmission
%=========================================================%
Rsym = 1e6;
Fsamp = Rsym*wg.SamplesPerSymbol;                                                % Sampling rate in samples per second

frameSize = getFrameSize(wg.FECFrame,wg.MODCOD)*wg.SamplesPerSymbol;
radioTx = sdrtx("Pluto");
radioTx.RadioID               = "usb:0";
radioTx.CenterFrequency       = txsim.RadioCenterFrequency;
radioTx.BasebandSampleRate    = Fsamp;
radioTx.SamplesPerFrame       = frameSize;
radioTx.Gain                  = txsim.RadioGain;

numRepeatTxFrames = 10000;                                                     % Waveform is repeatedly transmitted in a loop
disp(radioTx)
fprintf("Starting transmission at Fs = %g MHz\n",Fsamp/1e6)

for frame = 1:numRepeatTxFrames
   underrun = radioTx(txOut);
    if (underrun) 
        warning("Dropped samples.")
    end
end
fprintf("Transmission finished\n")
release(radioTx)
    
release(wg)

%=========================================================%
% Local function
%=========================================================%

function dfl = getDFL(modCod,fecFrame)
% Get data field length
if strcmp(fecFrame,"normal")
    nDefVal = [16008 21408 25728 32208 38688 43040 48408 51648 53840 57472 ...
        58192 38688 43040 48408 53840 57472 58192 43040 48408 51648 53840 ...
        57472 58192 48408 51648 53840 57472 58192] - 80;
else
    nDefVal = [3072 5232 6312 7032 9552 10632 11712 12432 13152 14232 0 ...
        9552 10632 11712 13152 14232 0 10632 11712 12432 13152 14232 0 11712 ...
        12432 13152 14232 0] - 80;
end
dfl = nDefVal(modCod);
end
function frameSize = getFrameSize(fecFrame,modCod)
% Get PL frame size
[modOrder,~,cwLen] = satcom.internal.dvbs.getS2PHYParams(modCod,fecFrame);
dataLen = cwLen/log2(modOrder);
% Pilot sequence and indices generation
slotLen = 90;
pilotBlkFreq = 16;                                                              % In slots
numPilotBlks = floor(dataLen/(slotLen*pilotBlkFreq));
if floor(dataLen/(slotLen*16)) == dataLen/(slotLen*pilotBlkFreq)
    numPilotBlks = numPilotBlks - 1;
end
pilotLen = numPilotBlks*36;                                                     % One pilot block contains 36 pilot symbols
frameSize = dataLen + pilotLen + slotLen;
end