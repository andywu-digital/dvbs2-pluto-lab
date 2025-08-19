function R = HelperDVBS2FineFreqEst(rxPilots, numPilotBlks, refPilots, R, varargin)
%HelperDVBS2FineFreqEst Estimate N point auto correlation value for fine
%frequency error calculation using pilot blocks in DVB-S2/S2X PL frames
%
%   Note: This is a helper function and its API and/or functionality may
%   change in subsequent releases.
%
%   R = HelperDVBS2FineFreqEst(RXPILOTS,NUMPILOTBLKS,REFPILOTS,R)
%   estimates the N point auto correlation function over pilot fields
%   using received pilots, RXPILOTS, number of pilot blocks in the PL
%   frame, NUMPILOTBLKS, PL scrambled reference pilots, REFPILOTS and
%   previous auto correlation estimate, R based on the L&R estimation
%   algorithm[3]. The auto correlation is performed over many pilot fields
%   to achieve the accuracy required.
% 
%   R = HelperDVBS2FineFreqEst(RXPILOTS, NUMPILOTBLKS, REFPILOTS, R, ...
%   NUMBLKS, N) allows other pilot block lengths like 34 or 32 (VL-SNR
%   additional pilot fields) using the first optional argument. The N value
%   used for auto-correlation of regular frames is 18 and for VL-SNR
%   frames, it can be provided as the second optional input argument. The
%   estimator can track a frequency error up to 5 percent of input symbol
%   rate if N is 18 (default) and 1/(N+1) when N is provided as optional
%   argument[3].
%
%   References:
%   [1] E. Casini, R. De Gaudenzi and A. Ginesi: "DVB-S2
%   modem algorithms design and performance over typical satellite
%   channels", International Journal on Satellite Communication
%   Networks, Volume 22, Issue 3.
%   [2] Umberto Mengali and Aldo N. D'Andrea, Synchronization
%   Techniques for Digital Receivers. New York: Plenum Press, 1997.
%   [3] Luise, M. and R. Regiannini. "Carrier recovery in all-digital
%   modems for burst-mode transmissions.” IEEE Transactions on
%   Communications. Vol. 43, No. 2, 3, 4, Feb/Mar/April, 1995,
%   pp. 1169–1178.

%   Copyright 2020-2021 The MathWorks, Inc.

if isempty(varargin) % For regular PL frames
    Lp = 36; % Pilot block length
    N = 18; % The number of elements used to compute the auto correlation over each
            % pilot block
else % For VL-SNR frames
    Lp = varargin{1}; % Can be one of 36, 34 and 32 lengths
    N = varargin{2};
end

pilotBlock = reshape(rxPilots, Lp, numPilotBlks);
refBlock = reshape(refPilots, Lp, numPilotBlks);

% Rm(m) = (sum(z(k)*conj(c(k))*conj(z(k-m))*c(k-m)))/(Lp-m)
% The summation is performed over Lp pilot symbols.

% The auto correlation is averaged over multiple pilot blocks.
for n = 1:numPilotBlks
    Rm = complex(zeros(N, 1));
    for m = 1:N
        for k = m+1:Lp
            Rm(m) = Rm(m) + pilotBlock(k, n).*conj(refBlock(k, n))...
                .*conj(pilotBlock(k-m, n)).*refBlock(k-m, n);
        end
        Rm(m) = Rm(m)/(Lp-m);
    end
    R = R + sum(Rm);
end
