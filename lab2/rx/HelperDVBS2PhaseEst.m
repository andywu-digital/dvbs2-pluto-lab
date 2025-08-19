function [phErrEst, prevPhErrEst] = HelperDVBS2PhaseEst(rxData, ...
    refPilots, prevPhErrEst, varargin)
%HelperDVBS2PhaseEst Estimate residual carrier frequency and phase error
%using pilot blocks in DVB-S2/S2X frames
%
%   Note: This is a helper function and its API and/or functionality may
%   change in subsequent releases.
%
%   [PHERREST,PREVPHERREST] = HelperDVBS2PhaseEst(RXDATA,REFPILOTS, ...
%   PREVPHERREST,PILOTIND) estimates the residual frequency error and
%   carrier phase error using the start of frame (SOF) and pilots in the
%   received data, RXDATA, PL scrambled reference pilots, REFPILOTS. The
%   phase estimated on consecutive pilot slots are used to compensate for
%   the data portion of the slot. The phase estimated on the last pilot
%   block of previous frame, PREVPHERREST is used in unwrapping the phase
%   estimates computed on the current PL frame. The residual carrier
%   frequency error should be in the order of 2e-4 of symbol rate for the
%   estimation to be accurate.
%
%   [PHERREST, PREVPHERREST] = HelperDVBS2PhaseEst(RXDATA, REFPILOTS, ...
%   PREVPHERREST, ISVLSNR, SETNUM, ALPHA) estimates the residual frequency
%   error and carrier phase error for VL-SNR frames using the
%   additional information from VL-SNR set number, SETNUM and slope used in
%   unwrapping the estimate, ALPHA. VL-SNR set number is used for
%   identifying the different pilot structure between 
%
%   References:
%   E. Casini, R. De Gaudenzi and A. Ginesi: "DVB-S2
%   modem algorithms design and performance over typical satellite
%   channels", International Journal on Satellite Communication
%   Networks, Volume 22, Issue 3.

%   Copyright 2020-2023 The MathWorks, Inc.

if nargin == 4
    pilotInd = varargin{1};
    isVLSNR = false;
    setNum = 0;
    alpha = 1;
    rxPilots = rxData(pilotInd);
else
    isVLSNR = varargin{1};
    setNum = varargin{2};
    alpha =  varargin{3};
    rxPilots = rxData;
end
if isVLSNR
    if setNum == 1 % VL-SNR set 1 pilot structure
        blkLens = zeros(43,1);
        blkLens(1:2:end) = 36;
        blkLens(2:2:36) = 34;
        blkLens(38:2:end) = 36;
    else % VL-SNR set 2 pilot structure
        blkLens = zeros(21,1);
        blkLens(1:2:end) = 36;
        blkLens(2:2:18) = 32;
        blkLens(20) = 36;
    end
    numBlks = numel(blkLens);
    prevPhErrEst = 0;
    phErrEst = zeros(numBlks+1, 1);
    off = 1;
else  % Regular PL frames
    blkLen = 36; % Number of pilots in a pilot block
    numBlks = length(rxPilots)/blkLen;
    blkLens = ones(numBlks,1)*blkLen;
    y = satcom.internal.dvbs.plHeader('S2',1,false,64800);
    refSOF = y(1:26);
    phTemp = angle(sum(rxData(1:26).*conj(refSOF)));
    phErrEst = zeros(numBlks+2, 1);
    phErrEst(2) = prevPhErrEst + wrapToPi(phTemp-prevPhErrEst);
    prevPhErrEst = phErrEst(2);
    off = 2;
end
stIdx = 0;
phErrEst(1) = prevPhErrEst;
for idx = 1:numBlks
    endIdx = stIdx+blkLens(idx);
    winLen = stIdx+1:endIdx;
    buffer = rxPilots(winLen);
    ref = refPilots(winLen);
    % carrier phase error calculation
    phTemp = angle(sum(buffer.*conj(ref)));
    % Unwrapping the phase error using the phase estimate made on the
    % previous pilot block
    phErrEst(idx+off) = prevPhErrEst + alpha*wrapToPi(phTemp-prevPhErrEst);
    prevPhErrEst = phErrEst(idx+off);
    stIdx = endIdx;
end
end
