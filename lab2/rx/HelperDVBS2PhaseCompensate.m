function syncOut = HelperDVBS2PhaseCompensate(rxData, phEst, ...
    pilotInd, arg4, varargin)
%HelperDVBS2PhaseCompensate Compensate residual frequency and phase error
%in DVB-S2/S2X PL frames
%
%   Note: This is a helper function and its API and/or functionality may
%   change in subsequent releases.
%
%   SYNCOUT = HelperDVBS2PhaseCompensate(RXDATA,PHEST,PILOTIND,BLK1EST)
%   compensates the residual carrier frequency and phase error on the
%   received PL frame, RXDATA using the phase estimated on pilot blocks of
%   the current frame, PHEST and the phase estimated on the first pilot
%   block of next frame, BLK1EST and pilot block indices, PILOTIND.
% 
%   SYNCOUT = HelperDVBS2PhaseCompensate(RXDATA, PHEST, PILOTIND, SETNUM,
%   MODENUM) compensates the residual carrier frequency and phase error
%   on the received PL frame, RXDATA using the phase estimated on pilot
%   blocks of the current frame, PHEST, and the VL-SNR set number i.e
%   either 1 or 2 for VL-SNR mode (MODENUM = 1) and wideband mode
%   (MODENUM = 2).
% 
%   SYNCOUT is the carrier frequency and phase compensated PL frame. The
%   phase estimated on the consecutive pilot blocks are interpolated to
%   compensate for the phase error on the data slot.
%
%   References: 
%   E. Casini, R. De Gaudenzi and A. Ginesi: "DVB-S2 modem algorithms
%   design and performance over typical satellite channels", International
%   Journal on Satellite Communication Networks, Volume 22, Issue 3.

%   Copyright 2020-2023 The MathWorks, Inc.
isReg = true;
if ~isempty(varargin)
    isReg = false;
    isVLSNR =  varargin{1} == 1;
end
if isReg
    blkLen = 36; % Number of pilots in a pilot block
    numBlks = length(pilotInd)/blkLen;
    syncOut = zeros(size(rxData));
    pilotBlkFreq = 1476; %In symbols
    chunkLLen = length(rxData)-pilotInd(end);
    % The number of data symbols between the last pilot field of previous frame
    % and first pilot field of current frame is different due to header
    % insertion in between. That is why the compensation of header and data
    % portion before the first pilot block are handled independently.
    phData =  phEst(2)+(phEst(3)-phEst(2)).*(1:1566)/1566;

    % 1566 corresponds to pilotBlkFreq + PL header length (90 symbols).
    syncOut(1:1566) = rxData(1:1566).*exp(-1j*phData(:));
    endIdx = pilotInd(blkLen);

    % Compensation for all data slots which are uniformly separated by pilot
    % fields.

    for idx = 2:numBlks
        stIdx = endIdx+1;
        endIdx = pilotInd(idx*blkLen);
        % Interpolation of phase estimate on data using the phase estimates
        % computed on preceding and succeeding pilot blocks
        phData = phEst(idx+1)+(phEst(idx+2)-phEst(idx+1)).*(1:pilotBlkFreq)/pilotBlkFreq;
        syncOut(stIdx:endIdx) = rxData(stIdx:endIdx).*exp(-1j*phData(:));
    end
    phData = phEst(end)+(arg4-phEst(end)).*(1:chunkLLen)/chunkLLen;
    syncOut(pilotInd(end)+1:end) = rxData(pilotInd(end)+1:end).*exp(-1j*phData(:));
elseif isVLSNR
    setNum = arg4;
    % ETSI 302 307-2 Section 5.5.2 Figures 17 and 18
    if setNum == 1
        blkLens = zeros(43,1);
        blkLens(1:2:end) = 36;
        blkLens(2:2:36) = 34;
        blkLens(38:2:end) = 36;
        period = [540;703.*ones(36,1);702.*ones(6,1)];
        pilotBlkFreq = period+blkLens;
        chunk1Len = 540+36;
        chunkNLen = 720;
    else
        blkLens = zeros(21,1);
        blkLens(1:2:end) = 36;
        blkLens(2:2:18) = 32;
        blkLens(20) = 36;
        period = [540;704.*ones(18,1);702.*ones(2,1)];
        pilotBlkFreq = period+blkLens;
        chunk1Len = 540+36;
        chunkNLen = 360;
    end
    % blkLen = 36; % Number of pilots in a pilot block
    numBlks = numel(blkLens);
    syncOut = zeros(size(rxData));
    endIdx = pilotInd(36);

    % Compensation for all data slots which are uniformly separated by pilot
    % fields.
    pIdx = blkLens(1);
    baseInd = zeros(numBlks,1);
    for idx = 2:numBlks
        stIdx = endIdx+1;
        endIdx = pilotInd(pIdx+blkLens(idx));
        % Interpolation of phase estimate on data using the phase estimates
        % computed on preceding and succeeding pilot blocks
        phData = phEst(idx)+(phEst(idx+1)-phEst(idx)).*(1:pilotBlkFreq(idx))/pilotBlkFreq(idx);
        syncOut(stIdx:endIdx) = rxData(stIdx:endIdx).*exp(-1j*phData(:));
        pIdx = pIdx+blkLens(idx);
        baseInd(idx-1) = stIdx;
    end
    baseInd(end) = endIdx;
    firstEst = interp1(baseInd,phEst(2:end),1,'linear','extrap');
    phData =  firstEst+(phEst(2)-firstEst).*(1:chunk1Len)/(chunk1Len);
    syncOut(1:chunk1Len) = rxData(1:chunk1Len).*exp(-1j*phData(:));
    lastEst = interp1(baseInd,phEst(2:end),endIdx+chunkNLen,'linear','extrap');
    phData = phEst(end)+(lastEst-phEst(end)).*(1:chunkNLen)/chunkNLen;
    syncOut(pilotInd(end)+1:end) = rxData(pilotInd(end)+1:end).*exp(-1j*phData(:));
else
    blkLen = 36; % Number of pilots in a pilot block
    numBlks = length(pilotInd)/blkLen;
    syncOut = zeros(size(rxData));
    pilotBlkFreq = 1476; %In symbols
    chunkNLen = length(rxData)-pilotInd(end);
    % Compensation for all data slots which are uniformly separated by pilot
    % fields.
     phData =  phEst(2)+(phEst(3)-phEst(2)).*(1:1656)/1656;
    % 1566 corresponds to pilotBlkFreq + PL header length (90 symbols).
    syncOut(1:1656) = rxData(1:1656).*exp(-1j*phData(:));
    baseInd = zeros(numBlks,1);
    endIdx = pilotInd(blkLen);
    for idx = 2:numBlks
        stIdx = endIdx+1;
        endIdx = pilotInd(idx*blkLen);
        % Interpolation of phase estimate on data using the phase estimates
        % computed on preceding and succeeding pilot blocks
        phData = phEst(idx+1)+(phEst(idx+2)-phEst(idx+1)).*(1:pilotBlkFreq)/pilotBlkFreq;
        syncOut(stIdx:endIdx) = rxData(stIdx:endIdx).*exp(-1j*phData(:));
        baseInd(idx-1) = stIdx;
    end
    baseInd(end) = endIdx;
    lastEst = interp1(baseInd,phEst(3:end),endIdx+chunkNLen,'linear','extrap');
    phData = phEst(end)+(lastEst-phEst(end)).*(1:chunkNLen)/chunkNLen;
    syncOut(pilotInd(end)+1:end) = rxData(pilotInd(end)+1:end).*exp(-1j*phData(:));
end
end