% This file configurates the parameters used in the program
% Author: Sean Quirin, modified by Weijian Yang 2014-2018

%% SLM parameters
SLM='BNS';
%SLM='Holoeye';

if strcmp(SLM, 'BNS')           % BNS SLM
    SLMm=512;           % pixel number in the horizontal axis of the SLM
    SLMn=512;           % pixel number in the vertical axis of the SLM
    SLMLoadTime=0.05;    % SLM load time
% the default linear LUT file name for BNS SLM
    defaultLUTfileName='C:\BLINK_PCIe\LUT_Files\linear.LUT';                           
% the calibrated LUT file name for BNS SLM 
    calibratedLUTfileName='C:\BLINK_PCIe\LUT_Files\gamma_cal_1040nm_BNS.lut';   
% when initializing the BNS SLM, the current directory should be the following one
    BNSSLMInitializationPath='C:\PCIeMatlabSDK';              
% the file that is used to optimize the BNS SLM 
    SLMOptimizationDataFileName='C:\BLINK_PCIe\Img\blank.bmp';
% default loaded image when initialize the BNS SLM
    SLMInitializationImageFileName='C:\BLINK_PCIe\Img\blank.bmp';
    SLMInitializationImageMat='C:\BLINK_PCIe\Img\AOPhase.mat';
    SLMInitializationImageMat=[];
else                     % Holoeye SLM
    SLM_M=1920;          % total pixel number in the horizontal axis of the SLM
    SLM_N=1080;          % total pixel number in the vertical axis of the SLM; the extra number is to make the screen larger so as to avoid SLM blinking
    SLMm=1080;           % pixel number in the horizontal axis of the SLM that are to be illuminated
    SLMn=1080;           % pixel number in the vertical axis of the SLM that are to be illuminated
    SLMm0=1;             % pixel coordinate in the horizontal axis, representing the bottom left of the image 
    SLMn0=1;             % pixel coordinate in the vertical axis, representing the bottom left of the image
    SLMLoadTime=0.1;     % SLM load time
% the default linear LUT file name for Holoeye SLM
    defaultLUTfileName='C:\BLINK_PCIe\LUT_Files\linear.LUT';                           
% the calibrated LUT file name for Holoeye SLM 
    calibratedLUTfileName='C:\BLINK_PCIe\LUT_Files\gamma_cal_1040nm_Holoeye.lut';
    calibratedLUTFunctionfileName='C:\BLINK_PCIe\LUT_Files\gamma_cal_1040nm_Holoeye.mat';
end

SLMPreset=[0 -9];   % [0 12];
objectiveRI=1.33;
illuminationWavelength=1040e-9;
