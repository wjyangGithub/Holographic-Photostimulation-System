function [ handles ] = BNS_SLM_Initialize(LUTfileName)
%--------------------------------------------------------------------------
%--- Sean Quirin, 2012.06.25
%--- modified the BNS SDK to suit three separate command line functions.
%--- this function is to start up the SLM and return the handles to control
%--- it.

%--- Modified by Weijian Yang, 2014.04
%--------------------------------------------------------------------------

% Load the configuration file
    CFG;

% To initialize the BNS SLM, one has to be in a specific path, indicated by
% the "BNSSLMInitializationPath"
    currentPath=pwd;
    cd(BNSSLMInitializationPath);

% Call the constructor, which opens a handle to the hardware and 
% returns the number of PCIe boards found in the computer    
    handles.NumDevices = BNS_OpenSLMs();

% Go back the current directory    
    cd(currentPath);
        
% Load a LUT file to each PCIe board
    for SLM = 1:handles.NumDevices
        if(SLM == 1)
            BNS_LoadLUTFile(1,LUTfileName);   
        else
            BNS_LoadLUTFile(2,defaultLUTfileName);
        end
    end
    BNS_SetPower(true);                
% Read in the wavefront correction file for each SLM
    handles.optimization_data1 = double(imread(SLMOptimizationDataFileName,'bmp'));
    handles.apply_optimization = true;
    handles.num_images = 1;
    image = double(imread(SLMInitializationImageFileName,'bmp'));
% Initalize the data on the SLM
    BNS_SLM_LoadImage(image, handles);
end    
    
function NumBoards = BNS_OpenSLMs()   
%==========================================================================
%=   FUNCTION:  BNS_OpenSLMs()
%=
%=   PURPOSE: Opens all the Boulder Nonlinear Systems SLM driver boards 
%=            in the system.  Assumes the devices are nematic (phase) 
%=            SLMs.  Loads the library "Interface.dll" into the MATLAB
%=            workspace.
%=            
%=   OUTPUTS: 
%=
%==========================================================================
    loadlibrary('C:\PCIeMatlabSDK\Interface.dll', @BNSPCIeInterface, 'alias', 'SLMlib');
    TrueFrames = int32(10);           % 22
    %NumBoards = calllib('SLMlib', 'Constructor', 0, TrueFrames, 'PCIe256');
    NumBoards = calllib('SLMlib', 'Constructor', 0, TrueFrames, 'PCIe512');
end    

function BNS_LoadLUTFile(board, LUTFileName)
%==========================================================================
%=   FUNCTION: BNS_LoadLUTFile(LUTFileName)
%=
%=   PURPOSE: Calls a C++ sub-function to read and loads the LUT to the
%             hardware
%=
%=   INPUTS:  The PCIe board that the LUT is being written to, and the 
%=            LUT File Name
%=
%=   OUTPUTS:  none
%=
%==========================================================================
    %LUTFileName
    calllib('SLMlib', 'LoadLUTFile', board, LUTFileName);   
end
    
function BNS_SetPower(bPower)
%==========================================================================
%=   FUNCTION: BNS_SetPower(bPower)
%=
%=   PURPOSE: Toggles the SLM power state
%=
%=   INPUTS:  a boolean state - true = power up, false = power down
%=
%=  OUTPUTS:  
%=
%==========================================================================  
    calllib('SLMlib', 'SLMPower', bPower);  
end