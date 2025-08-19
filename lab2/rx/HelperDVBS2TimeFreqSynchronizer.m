classdef HelperDVBS2TimeFreqSynchronizer < matlab.System
    %HelperDVBS2TimeFreqSynchronizer DVB-S2/S2X symbol timing, frame and
    %coarse frequency synchronizer
    %
    %   Note: This is a helper function and its API and/or functionality
    %   may change in subsequent releases.
    %
    %   HSYNC = HelperDVBS2TimeFreqSynchronizer creates a symbol timing,
    %   frame and coarse frequency synchronizer object, HSYNC, that
    %   compensates for timing phase and carrier frequency offsets and
    %   detects the starting index of physical layer frame. The object is
    %   designed to work for DVB-S2/S2X physical layer (PL) frames in pilot
    %   aided mode. This object uses Gardner timing error detector for
    %   compensating the timing offsets and produces output at symbol rate.
    %   Differential correlation is performed on PL header for identifying
    %   the start of frame. This object uses pilot aided frequency locked
    %   loop (FLL) approach to reduce frequency offset. The object can
    %   track a frequency error up to 25 percent of input symbol rate.
    %   Matched filtering and decimation to required samples per symbol for
    %   timing error correction are performed internal to the loop.
    %
    %   Step method syntax:
    %
    %   [OUT, SYNCINDEX, PHEST] = step(HSYNC, IN, FREQLOCK) estimates the
    %   timing phase and carrier frequency offset for each sample in the
    %   input, IN of the input, IN and returns the synchronized signal,
    %   OUT, the frame starting index, SYNCINDEX and the estimated phase
    %   offset, PHEST. IN must be a column vector. The input signal
    %   sampling rate must be at least twice the symbol rate. The step
    %   method outputs the phase corrected OUT at the symbol rate. The
    %   phase estimate will be of the same size as the input. When FREQLOCK
    %   is set to true, the FLL is run by a reduced loop bandwidth of 10.
    %
    %   HelperDVBS2TimeFreqSynchronizer properties:
    %
    %   CarrSyncLoopBW           - Loop bandwidth used by FLL normalized by
    %                              symbol rate
    %   SymbSyncLoopBW           - Loop bandwidth used by symbol timing
    %                              synchronizer normalized by symbol rate
    %   SamplesPerSymbol         - Number of samples representing each
    %                              symbol
    %   RolloffFactor            - Roll-off factor for receive filtering
    %   DataFrameSize            - PL data frame length
    %   SymbSyncTransitFrames    - Number of frames required for symbol
    %                              synchronizer to converge
    %   FrameSyncAveragingFrames - Number of frames required for averaging
    %                              out the frame start index to an error
    %                              probability of less than 1%.
    %   PLScramblingIndex        - Physical layer scrambling index
    %
    %   References:
    %   [1] E. Casini, R. De Gaudenzi and A. Ginesi: "DVB-S2
    %   modem algorithms design and performance over typical satellite
    %   channels", International Journal on Satellite Communication
    %   Networks, Volume 22, Issue 3.
    %   [2] Umberto Mengali and Aldo N. D'Andrea, Synchronization
    %   Techniques for Digital Receivers. New York: Plenum Press, 1997.

    %   Copyright 2020-2022 The MathWorks, Inc.

    properties
        %CarrSyncLoopBW Normalized loop bandwidth for FLL
        %   Specify normalized loop bandwidth for FLL as a real positive
        %   scalar. The default value is 1e-4*0.023. Decreasing the loop
        %   bandwidth increases the convergence time, and increases the
        %   estimation accuracy. The frequency error is computed on the
        %   pilots which are periodically placed between data symbols and
        %   it is locked during the data portion. 0.023 factor accounts for
        %   the pilot periodicity.
        CarrSyncLoopBW = 1e-4;
        %SymbSyncLoopBW Normalized loop bandwidth for symbol timing
        %synchronizer
        %   Specify normalized loop bandwidth for symbol synchronizer as a
        %   real positive scalar. The default value is 1e-4. Decreasing the
        %   loop bandwidth increases the convergence time, and increases
        %   the estimation accuracy. The frequency error is computed on the
        %   pilots which are periodically placed between data symbols and
        %   it is locked during the data portion.
        %   SymbSyncLoopBW/SamplesPerSymbol must be always greater than
        %   1e-5 for proper convergence.
        SymbSyncLoopBW = 1e-4;
    end
    properties (Nontunable)
        %SamplesPerSymbol Samples per symbol
        %   Specify the number of samples that represent each symbol in the
        %   input signal as an integer-valued real positive scalar greater
        %   than or equal to 4. The default is 4.
        SamplesPerSymbol = 4;
        %RolloffFactor Roll-off factor
        %   Specify the roll-off factor as one of 0.35 | 0.25 | 0.2 for
        %   receive RRC filter. The default is 0.35.
        RolloffFactor = 0.35;
        %DataFrameSize Data frame size
        %   Specify the data frame size of the DVB-S2/S2X physical layer
        %   data frame as a real positive scalar. The value must be
        %   computed based on the FEC frame type and modulation order used.
        %   For 'normal' frame, the value is defined as
        %   64800/log2(ModulationOrder) and for 'short' frame, it is
        %   defined as 16200/log2(ModulationOrder). The default value is
        %   32400 which corresponds to QPSK normal frame.
        DataFrameSize = 32400;
        %SymbSyncTransitFrames Number of frames required for symbol
        %synchronizer convergence
        %   Specify the number of frames required for symbol synchronizer
        %   convergence as a real positive scalar. The default value is 6
        %   which corresponds to QPSK normal frame.
        SymbSyncTransitFrames = 6;
        %FrameSyncAveragingFrames Number of frames required for getting
        %accurate frame start index
        %   Specify the number of frames required for getting accurate
        %   frame start index as a real positive scalar. The default value
        %   is 3 which corresponds to QPSK normal frame.
        FrameSyncAveragingFrames = 3;
        %PLScramblingIndex Physical layer scrambling index
        %    Specify the physical layer scrambling sequence index as an
        %    integer scalar in the range [0,7]. The gold sequence index
        %    used is PLScramblingIndex*10949 as per ETSI EN 302 307-2
        %    Section 5.5.4 Table 19e.
        PLScramblingIndex = 0;
    end

    properties (Access = private, Nontunable)
        % PL scrambled pilot sequence in each PL frame
        pPilotSeq
        % Input data type
        pInputDataType
        % Pilot indices in PL frame
        pPilotIndices
        % Symbol synchronizer object
        pSymbolSync
        % PL frame size
        pFullFrameSize
        % Coefficients of raised cosine filter
        pFilterCoefficients
    end

    properties (Access = private)
        % Integrator gain used in loop filter
        pIntegratorGain
        % Digital synthesizer gain
        pDigitalSynthesizerGain
        % Phase error estimate at input sample rate
        pPhase
        % Variable to store received pilot symbol - Used in freq error
        % calculation
        pPreviousSample
        % Variable to store pilot symbol delayed by two units - Used in freq
        % error calculation
        pPreviousSample2
        % Loop filter state variable
        pLoopFilterState
        % Integrator state variable
        pIntegFilterState
        % DDS state variable
        pDDSPreviousInput
        % Frequency error estimate
        pFreqError
        % PL frame counter
        pFrameCount
        % Buffer to hold symbols for frame synchronization
        pFSBuffer
        % Frame start index
        pSyncIndex = 1
        % Possible frame start indices- Used in averaging over the frames for
        % low SNR
        pPossibleSyncIndices
        % Buffer to store last 90 symbols of a frame
        pFrameBuffer        
        % Matched filter state
        pRRCPrevState
    end

    methods
        % Constructor
        function obj = HelperDVBS2TimeFreqSynchronizer(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        % Check NoiseBandwidth
        function set.CarrSyncLoopBW(obj, value)
            obj.CarrSyncLoopBW = value;
        end
        function set.SamplesPerSymbol(obj, value)
            obj.SamplesPerSymbol = value;
        end
        function set.RolloffFactor(obj, value)
            obj.RolloffFactor = value;
        end
        function set.SymbSyncLoopBW(obj, value)
            obj.SymbSyncLoopBW = value;
        end
        function set.DataFrameSize(obj, value)
            obj.DataFrameSize = value;
        end
        function set.SymbSyncTransitFrames(obj, value)
            obj.SymbSyncTransitFrames = value;
        end
        function set.FrameSyncAveragingFrames(obj, value)
            obj.FrameSyncAveragingFrames = value;
        end
    end

    methods (Access=protected)
        % Initial Object Setup
        function setupImpl(obj, input)
            obj.pInputDataType = class(input);
            % Get loop gains
            obj.pIntegratorGain = 4*obj.CarrSyncLoopBW*0.023;
            % Invert DDS output to correct not estimate
            obj.pDigitalSynthesizerGain = cast(-1,obj.pInputDataType);
            % Receive filter
            obj.pFilterCoefficients = rcosdesign(obj.RolloffFactor, 20, obj.SamplesPerSymbol);            
            % Save filter initial state
            obj.pRRCPrevState = zeros(length(obj.pFilterCoefficients)-1,1);
            % Initialize the symbol synchronizer object
            a = obj.RolloffFactor;
            Kp = 4*sin(pi*a/2)/(1-(a^2)/4);
            obj.pSymbolSync = comm.SymbolSynchronizer( ...
                'TimingErrorDetector','Gardner (non-data-aided)', ...
                'NormalizedLoopBandwidth',obj.SymbSyncLoopBW, ...
                'DampingFactor',1/sqrt(2), ...
                'DetectorGain',Kp, ...
                'SamplesPerSymbol',obj.SamplesPerSymbol);
            % Reference pilot field generation
            slotLen = 90;
            plScrambIntSeq = satcom.internal.dvbs.plScramblingIntegerSequence(obj.PLScramblingIndex);
            cMap = [1 1j -1 -1j].';
            cSeq = cMap(plScrambIntSeq+1);
            numPilots = floor(obj.DataFrameSize/(slotLen*16));
            if floor(obj.DataFrameSize/(slotLen*16)) == obj.DataFrameSize/(slotLen*16)
                numPilots = numPilots-1;
            end
            [~,obj.pPilotIndices] = satcom.internal.dvbs.pilotBlock(numPilots);
            obj.pPilotIndices = obj.pPilotIndices+90;
            obj.pPilotSeq = (1+1j)/sqrt(2).*cSeq(obj.pPilotIndices-90);
            obj.pFullFrameSize = obj.DataFrameSize+90+numPilots*36;
        end
        % Runtime Operation
        function [output, syncInd, phaseEst] = stepImpl(obj, input,...
                freqLock)

            % Copying to local variables for performance
            loopFiltState = obj.pLoopFilterState;
            integFiltState = obj.pIntegFilterState;
            DDSPreviousInp = obj.pDDSPreviousInput;
            prevSample = obj.pPreviousSample;
            prevSample2 = obj.pPreviousSample2;
            freqError = obj.pFreqError;
            sps = obj.SamplesPerSymbol;
            pIndices = obj.pPilotIndices;
            fBuffer = obj.pFSBuffer;
            symbolCount = 0;
            possSyncInd = obj.pPossibleSyncIndices;
            syncInd = obj.pSyncIndex;
            % Reduce the loop bandwidth by 10 when FLL has converged.
            if freqLock
                obj.pIntegratorGain = obj.pIntegratorGain/10;
            end
            % Preallocate outputs
            % Consider the case of samples insertion during symbol timing
            % synchronization
            output         = coder.nullcopy(zeros(size(input,1)/sps+11,1,'like',input));
            phaseCorrection = coder.nullcopy(zeros(size(input,1)+11*sps,1,'like',input));
            % Flag to indicate symbol timing PLL has converged.
            timeSyncStat   =  obj.pFrameCount > obj.SymbSyncTransitFrames;
            % Flag to indicate start of frame has been accurately detected and
            % coarse frequency offset tracking can start.
            frameSyncStat = obj.pFrameCount >  ...
                obj.SymbSyncTransitFrames + obj.FrameSyncAveragingFrames;
            numerator = obj.pFilterCoefficients;
            zf = obj.pRRCPrevState;
            gain = sum(obj.pFilterCoefficients);
            for k = 1:length(input)/sps
                winLen = (k-1)*sps+1:k*sps;
                % Matched filtering
                filtIn = input(winLen).*exp(1j*obj.pPhase);
                [filtOutTemp, zf] = filter(numerator, 1, filtIn, zf);
                filtOut = gain * (filtOutTemp);
                
                % Symbol synchronization output at symbol rate
                tSyncOut = obj.pSymbolSync(filtOut);
                nSymb = length(tSyncOut);
                for n = 1:nSymb
                    symbolCount = symbolCount+1;
                    if timeSyncStat
                        fBuffer(symbolCount) = tSyncOut(n);
                        if symbolCount == obj.pFullFrameSize-90
                            tempInd  =  HelperDVBS2FrameSync([obj.pFrameBuffer;fBuffer]);
                        end
                        if ~frameSyncStat
                            % Frame synchronization
                            if symbolCount == obj.pFullFrameSize-90
                                for z = 1:length(tempInd)
                                    p1 = find(possSyncInd(1,:) == tempInd(z), 1);
                                    p2 = find(possSyncInd(1,:) == tempInd(z)-1, 1);
                                    p3 = find(possSyncInd(1,:) == tempInd(z)+1, 1);
                                    if ~isempty(p1)
                                        possSyncInd(1,p1) = tempInd(z);
                                        possSyncInd(2,p1) = possSyncInd(2,p1)+1;
                                    elseif ~isempty(p2)
                                        possSyncInd(1,p2) = tempInd(z);
                                        possSyncInd(2,p2) = possSyncInd(2,p2)+1;
                                    elseif ~isempty(p3)
                                        possSyncInd(1,p3) = tempInd(z);
                                        possSyncInd(2,p3) = possSyncInd(2,p3)+1;
                                    else
                                        p4 = find(possSyncInd(2,:) == 0,1);
                                        possSyncInd(1,p4) = tempInd(z);
                                        possSyncInd(2,p4) = possSyncInd(2,p4)+1;
                                        % Check for dummy frame (dummy frame size is 3330)
                                        dummyIndex = abs(possSyncInd(1,p4)-possSyncInd(1,1:p4-1)) == 3330;
                                        if ~isempty(dummyIndex) && any(dummyIndex)
                                            possSyncInd(2,p4) = possSyncInd(2,p4)+possSyncInd(2,dummyIndex);
                                        end
                                    end
                                end
                                obj.pFSBuffer = zeros(obj.pFullFrameSize-90,1,obj.pInputDataType); % reset buffer
                                if obj.pFrameCount ==  ...
                                        obj.SymbSyncTransitFrames + obj.FrameSyncAveragingFrames
                                    [~,index] = max(possSyncInd(2,:));
                                    syncInd = possSyncInd(1,index(1))-90;
                                    if syncInd <= 0
                                        syncInd = obj.pFullFrameSize+syncInd;
                                    end
                                    possSyncInd = zeros(2,10,obj.pInputDataType);
                                end
                            end
                            
                        end
                        % Detect the pilots and calculate frequency error.
                        if frameSyncStat
                            tempPInd = mod(pIndices+syncInd-1,obj.pFullFrameSize);
                            tempPInd(tempPInd == 0) = obj.pFullFrameSize;
                            pilotInd = find(symbolCount == tempPInd);
                            idx = mod(pilotInd-1,36);
                            if ~isempty(pilotInd) && idx >= 3
                                freqError = ...
                                    imag(prevSample.*conj(prevSample2)*conj(obj.pPilotSeq(pilotInd-1))*obj.pPilotSeq(pilotInd-3));
                            end
                        end
                        output(symbolCount) = tSyncOut(n);
                    end
                end
                % First order loop filter implemented as an integrator
                loopFiltOut = freqError*obj.pIntegratorGain + loopFiltState;
                loopFiltState = loopFiltOut;
                % Direct digital synthesizer implemented as an integrator.
                % The estimation is done at symbol rate and the error is
                % corrected at input sampling rate
                for p = 1:sps
                    DDSOut = DDSPreviousInp + integFiltState;
                    integFiltState = DDSOut;
                    DDSPreviousInp = loopFiltState/sps;
                    obj.pPhase(p) = obj.pDigitalSynthesizerGain*DDSOut;
                end
                phaseCorrection(winLen) = obj.pPhase;
                for n = 1:nSymb
                    if timeSyncStat
                        if frameSyncStat
                            if ~isempty(pilotInd)
                                if tempPInd(pilotInd)-2 < 1
                                    prevSample2 = obj.pFrameBuffer(end+tempPInd(pilotInd)-2);
                                else
                                    prevSample2 = output(tempPInd(pilotInd)-2);
                                end
                                prevSample = output(tempPInd(pilotInd));
                            end
                        end
                    else
                        output(symbolCount) = tSyncOut(n);
                    end
                end
            end            
            obj.pRRCPrevState = zf;            
            if frameSyncStat
                % Track start of frame index after initial frame
                % synchronization
                if symbolCount > obj.pFullFrameSize-90
                    isdummy = tempInd-syncInd-90 == 3330;
                    iscont =  tempInd-syncInd-90 == 0; 
                    if any(isdummy) % Update the sync index if dummy frame is detected
                         syncInd = tempInd(isdummy)-90;
                    elseif any(iscont)  
                        syncInd = tempInd(iscont)-90;
                    else % New frame start index
                        for z = 1:length(tempInd)
                            p1 = find(possSyncInd(1,:) == tempInd(z), 1);
                            p2 = find(possSyncInd(1,:) == tempInd(z)-1, 1);
                            p3 = find(possSyncInd(1,:) == tempInd(z)+1, 1);
                            if ~isempty(p1)
                                possSyncInd(1,p1) = tempInd(z);
                                possSyncInd(2,p1) = possSyncInd(2,p1)+1;
                            elseif ~isempty(p2)
                                possSyncInd(1,p2) = tempInd(z);
                                possSyncInd(2,p2) = possSyncInd(2,p2)+1;
                            elseif ~isempty(p3)
                                possSyncInd(1,p3) = tempInd(z);
                                possSyncInd(2,p3) = possSyncInd(2,p3)+1;
                            else
                                p4 = find(possSyncInd(2,:) == 0,1);
                                possSyncInd(1,p4) = tempInd(z);
                                possSyncInd(2,p4) = possSyncInd(2,p4)+1;
                                dummyIndex = abs(possSyncInd(1,p4)-possSyncInd(1,1:p4-1)) == 3330;
                                if ~isempty(dummyIndex) && any(dummyIndex)
                                    possSyncInd(2,p4) = possSyncInd(2,p4)+possSyncInd(2,dummyIndex);
                                end
                            end
                        end
                        newSync = possSyncInd(2,possSyncInd(2,:) > obj.FrameSyncAveragingFrames/2) ;
                        if any(newSync)
                            [~,index] = max(newSync);
                             syncInd = possSyncInd(1,index(1))-90;
                             possSyncInd = zeros(2,10,obj.pInputDataType);
                             if syncInd <= 0
                                 syncInd = obj.pFullFrameSize+syncInd;
                             end
                        end
                    end 
                end
            end
            if obj.pFrameCount >=  ...
                    obj.SymbSyncTransitFrames + obj.FrameSyncAveragingFrames
                obj.pSyncIndex = mod(syncInd+obj.pFullFrameSize-symbolCount,obj.pFullFrameSize);
            end

            output = output(1:symbolCount);
            phaseEst = -real(phaseCorrection(1:winLen(end)));
            if length(output) > 89
                obj.pFrameBuffer = output(end-89:end);
            end
            %Updating states
            obj.pLoopFilterState = loopFiltState;
            obj.pIntegFilterState = integFiltState;
            obj.pPreviousSample = complex(prevSample);
            obj.pPreviousSample2 = complex(prevSample2);
            obj.pDDSPreviousInput = DDSPreviousInp;
            obj.pFrameCount = obj.pFrameCount+1;
            obj.pFreqError = freqError;
            obj.pPossibleSyncIndices = possSyncInd;
        end

        % Call after validation but before step when properties change
        function processTunedPropertiesImpl(obj)
            % Update loops when tunable properties change
            obj.pIntegratorGain = 4*obj.pCarrSyncLoopBW*0.023;
        end

        % Reset parameters and objects to initial states
        function resetImpl(obj)
            obj.pLoopFilterState = zeros(1,obj.pInputDataType);
            obj.pIntegFilterState = zeros(1,obj.pInputDataType);
            obj.pDDSPreviousInput = zeros(1,obj.pInputDataType);
            obj.pPhase = zeros(obj.SamplesPerSymbol,1,obj.pInputDataType);
            obj.pPreviousSample = complex(zeros(1,obj.pInputDataType));
            obj.pPreviousSample2 = complex(zeros(1,obj.pInputDataType));
            obj.pFreqError = zeros(1,obj.pInputDataType);
            obj.pFrameCount = zeros(1,obj.pInputDataType);
            obj.pFSBuffer = zeros(obj.pFullFrameSize-90,1,obj.pInputDataType);
            obj.pFrameBuffer = zeros(90,1,obj.pInputDataType);
            obj.pPossibleSyncIndices = zeros(2,10 ...
                ,obj.pInputDataType);
            obj.pSyncIndex = 1;            
            obj.pRRCPrevState = zeros(length(obj.pFilterCoefficients)-1,1);
            reset(obj.pSymbolSync)
        end

        % Release system object handles
        function releaseImpl(obj)                        
            obj.pRRCPrevState = zeros(length(obj.pFilterCoefficients)-1,1);
            release(obj.pSymbolSync)
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            s = saveObjectImpl@matlab.System(obj);
            if isLocked(obj)
                s.pPilotSeq                  =    obj.pPilotSeq;
                s.pInputDataType             =    obj.pInputDataType;
                s.pPilotIndices              =    obj.pPilotIndices;                
                s.pSymbolSync                =    obj.pSymbolSync;
                s.pFullFrameSize             =    obj.pFullFrameSize;
                s.pIntegratorGain            =    obj.pIntegratorGain;
                s.pDigitalSynthesizerGain    =    obj.pDigitalSynthesizerGain;
                s.pPhase                     =    obj.pPhase;
                s.pPreviousSample            =    obj.pPreviousSample;
                s.pPreviousSample2           =    obj.pPreviousSample2;
                s.pLoopFilterState           =    obj.pLoopFilterState;
                s.pIntegFilterState          =    obj.pIntegFilterState;
                s.pDDSPreviousInput          =    obj.pDDSPreviousInput;
                s.pFreqError                 =    obj.pFreqError;
                s.pFrameCount                =    obj.pFrameCount;
                s.pFSBuffer                  =    obj.pFSBuffer;
                s.pSyncIndex                 =    obj.pSyncIndex;
                s.pPossibleSyncIndices       =    obj.pPossibleSyncIndices;
                s.pFrameBuffer               =     obj.pFrameBuffer; 
            end
        end

        function loadObjectImpl(obj, s, wasLocked)
            % Set properties in object obj to values in structure s
            if wasLocked
                obj.pPilotSeq                  =    s.pPilotSeq;
                obj.pInputDataType             =    s.pInputDataType;
                obj.pPilotIndices              =    s.pPilotIndices;                
                obj.pSymbolSync                =    s.pSymbolSync;
                obj.pFullFrameSize             =    s.pFullFrameSize;
                obj.pIntegratorGain            =    s.pIntegratorGain;
                obj.pDigitalSynthesizerGain    =    s.pDigitalSynthesizerGain;
                obj.pPhase                     =    s.pPhase;
                obj.pPreviousSample            =    s.pPreviousSample;
                obj.pPreviousSample2           =    s.pPreviousSample2;
                obj.pLoopFilterState           =    s.pLoopFilterState;
                obj.pIntegFilterState          =    s.pIntegFilterState;
                obj.pDDSPreviousInput          =    s.pDDSPreviousInput;
                obj.pFreqError                 =    s.pFreqError;
                obj.pFrameCount                =    s.pFrameCount;
                obj.pFSBuffer                  =    s.pFSBuffer;
                obj.pSyncIndex                 =    s.pSyncIndex;
                obj.pPossibleSyncIndices       =    s.pPossibleSyncIndices;
                obj.pFrameBuffer               =    s.pFrameBuffer; 
            end
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

    end

    % Static Methods
    methods(Static,Access=protected)
        function groups = getPropertyGroupsImpl
            groups = matlab.system.display.Section(...
                'Title', 'Parameters',...
                'PropertyList', {'CarrSyncLoopBW', 'SymbSyncLoopBW', ...
                'SamplesPerSymbol', 'RolloffFactor', 'DataFrameSize', ...
                'SymbSyncTransitFrames', 'FrameSyncAveragingFrames'});
        end
    end
end