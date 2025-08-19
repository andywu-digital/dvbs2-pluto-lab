function nVarEst = HelperDVBS2NoiseVarEstimate(rxData, pilotInd, refPilots, normFlag)
%HelperDVBS2NoiseVarEstimate Estimate noise variance on DVB-S2/S2X pilot
%blocks
%
%   Note: This is a helper function and its API and/or functionality may
%   change in subsequent releases.
%
%   NVAREST = HelperDVBS2NoiseVarEstimate(RXDATA, PILOTIND, REFPILOTS, ...
%   NORMFLAG) estimates noise variance on the pilot symbols present in
%   received PL frame, RXDATA, using the pilot indices, PILOTIND, PL
%   scrambled reference pilots, REFPILOTS. The soft bits are calculated on
%   unit average power constellation in demodulator. The noise variance
%   estimate must be calculated w.r.t unit average scaled constellation. If
%   constellation normalization flag is set as false, the received PL frame
%   constellation will be normalized and noise variance will be estimated.
%   Data-aided maximum likelihood estimator (SNORE) is used.
%
%   Copyright 2020 The MathWorks, Inc.

if normFlag
    rxData = rxData/sqrt(mean(abs(rxData).^2));
end
rxPilots = rxData(pilotInd, :);
pSigN = mean(abs(rxPilots).^2);
% If received pilots and reference pilots are at the same amplitude
% scaling, abs() is enough to compute the signal power. If they are at different
% scaling levels, abs()^2 gives the appropriate scaling factor in the power
% calculation equation.
% mean((A+N)/K.*(A+N)/K) = A^2/K^2
% abs(mean((A+N)/K.*A)) = A^2/K
% abs(mean((A+N)/K.*A))^2 = A^4/K^2   A must be unit average scaled. 
pSig = abs(mean(rxPilots.*conj(refPilots))).^2;
nVarEst = abs(mean(pSigN-pSig));

end