function BNS_SLM_LoadImage(ImageMatrix, handles)
%==========================================================================
%=   FUNCTION:   BNS_SLM_LoadImage(ImageMatrix, handles)
%=
%=   PURPOSE:    Loads a image into a memory frame on the SLM driver board.
%=               WARNING - Loading the same memory frame that is currently
%=               being viewed on the SLM can result in corrupted images.  
%=             
%=   INPUTS:     ImageMatrix - A 512x512 matrix or 256x256 of integers, each 
%=                             within range 0..255, corresponding to the voltage
%=                             to be applied to the SLM pixel.
%=
%=   OUTPUTS:  
%=
%==========================================================================

% Loop through each PCI board found, and load an image
    for SLM=1:handles.NumDevices
        if(SLM == 1)
            wavefront = handles.optimization_data1;
        elseif(SLM == 2)
            wavefront = handles.optimization_data2;
        else
            wavefront = handles.optimization_data3;
        end
                
% Add our phase optimization if selected, modulo 256. The image
% will be processed through the LUT in the hardware
        if(handles.apply_optimization)
            ImageMatrix = mod(ImageMatrix + wavefront, 256);
        end;

% pass an array pointer down to the C++ code
        pImage = libpointer('uint8Ptr', ImageMatrix); 
        calllib('SLMlib', 'WriteImage', SLM, pImage, 512);
    end
end


