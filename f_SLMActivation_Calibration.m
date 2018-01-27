% Wrapper to calculate SLM hologram given the target points, and load to SLM 
% Author: Weijian Yang, 2015-2018

% to use this, uncomment line 30, and 36

function [ correctedPhase ] = f_SLMActivation_Calibration( SLM_handles, xyzp, weight, objectiveNA )
%SLM Summary of this function goes here
%   Detailed explanation goes here
    CFG;

% -------Create SLM phase pattern-------    
    phase=f_SLM_PhaseHologram( xyzp, SLMm, SLMn,  weight, objectiveNA, objectiveRI, illuminationWavelength );
    
% -------Add adaptive optics system correction
    if ~isempty(SLMInitializationImageMat)
        load(SLMInitializationImageMat);
        phase=phase+correctedWavefront;
    end

    phase=mod(phase,2*pi);
        
% -------SLM activation-------     
    if strcmp(SLM, 'BNS')
        phase = 255.*( phase )./(2*pi);            % BNS phase range 0~255
% for the BNS SLM, the bitmap which is loaded to
% the SLM must be pre-shifted. This is accomplished
% by padding the bmp, applying a circshift and then
% grabbing the center of the padded bmp
        correctedPhase = BNS_SLM_RegisterBMP(phase);
%        correctedPhase = phase;
% output to SLM and take image
%         BNS_SLM_LoadImage( correctedPhase, SLM_handles );
    else                                             
        phase = ( phase )./(2*pi);                 % Holoeye phase range 0~1  
        correctedPhase = phase';
%        Holoeye_SLM_LoadImage( correctedPhase, SLM_M, SLM_N, SLMm0, SLMn0, SLM_handles, calibratedLUTFunctionfileName );
    end
    pause(SLMLoadTime);
    
end

