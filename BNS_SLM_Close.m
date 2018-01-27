function [] = BNS_SLM_Close( handles )
%--------------------------------------------------------------------------
%--- Sean Quirin, 2012.06.25
%--- modified the BNS SDK to suit three separate command line functions.
%--- this function is to close the SLM after completing operations. It is
%--- last of the three main function calls
%--------------------------------------------------------------------------
    BNS_CloseSLM();
    %delete(handles.TIMER);
end    
        
function BNS_CloseSLM()
%==========================================================================
%=   FUNCTION:  BNS_CloseSLM()
%=
%=   PURPOSE:   Closes the Boulder Nonlinear Systems SLM driver boards
%=              and unloads Interface.dll from the MATLAB Workspace
%==========================================================================
   %loadlibrary('C:\PCIeMatlabSDK\Interface.dll', @BNSPCIeInterface);
   %calllib('Interface', 'SLMPower', 0);
   %calllib('Interface', 'Deconstructor');
   %unloadlibrary('Interface');
   calllib('SLMlib', 'SLMPower', 0);
   calllib('SLMlib', 'Deconstructor');
   unloadlibrary('SLMlib');
end