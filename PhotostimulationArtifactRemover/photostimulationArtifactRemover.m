% This program suppresses/removes the photostimulation artifacts due to photostimulation.
% This code takes in the raw recording and outputs corrected recording 
% (for individual corrected photostimulation session) where the photostimulation 
% artifacts are suppressed. An imageJ macro program is then used to replace
% the contanminated recording sessions with these corrected sessions. 

% Algorithm:
% Since the pixel rate (~8.2 MHz) of the calcium imaging recording is much 
% faster than the photostimulation laser’s pulse repetition rate (200 kHz ~ 1 MHz), 
% the fluorescence background appears to be a mesh grid shape in the calcium 
% imaging movie. To detect the pixels having this artifact, we consider both 
% their fluorescence value and their geometry. First we detect candidate pixels 
% by identifying pixels whose value is significantly higher from the average (or maximum) 
% value calculated from a few frames just before and just after the stimulation. 
% Second, these candidate pixels are tested for connectedness within every horizontal 
% and vertical line of each frame, and the width of the connections compared to 
% that expected based on the stimulation condition. If both these conditions hold, 
% these pixels are marked as ‘contaminated’ and the fluorescence value at these pixels 
% during the stimulation are replaced by those in their adjacent ‘clean’ pixels. 
% This pre-processing significantly suppresses the artifacts while maintaining the
% original signal.

% Author: Weijian Yang, 2015-2018

%% load the data
fileName='recordingWithStimArtifact.tif';      % recorded movie (for a specific plane) with photostimulation artifact
saveFileName='correctedRecording';             % new files to be saved where the photostimulation artifacts are removed for each individual photostimulation session (no need to add ".tif" here; the code will add it itself)
onset=load('photoStimulationOnset.txt');       % frames when each photostimulation starts
endset=load('photoStimulationEndset.txt');     % frames when each photostimulation ends

averageFrameNum=3;             % the number of frames just before and just after the stimulation, used to calculate the average (or maximum) pixel value of the "baseline".
artifactThr=1000;              % threshold to identify if the candidate pixel is contanminated (to see if its value is significantly higher from the average (or maximum) value calculated from a few frames just before and just after the stimulation.) 
artifactWidth=4;               % mesh grid width of the artifact (this depends on the setting of the pixel rate in recording, and the recording electronics). 
stimThr=500;                   % threshold used to identify which frames the photostimulation occurs in "stimTimingFile".

%% read the recording file 
section=length(onset);
newData=cell(1,section);
for idx=1:section            % cycle through each photostimulation
    data=bigread2(fileName,onset(idx)-averageFrameNum,endset(idx)-onset(idx)+2*averageFrameNum+1);   % extract the recording frames during each photostimulation, plus "averageFrameNum" before and after   
    data=double(data);
    data1=data(:,:,averageFrameNum+1:end-averageFrameNum);         % the recording frames during each photostimulation
    data0=max(data(:,:,[1:averageFrameNum end-averageFrameNum+1:end]),[],3);   % maximum value of the "averageFrameNum" of frames just before and just after the stimulation; one can consider to use average instead of maximum here. This serves as "baseline".
    frameNum=endset(idx)-onset(idx)+1;                             % total frame number for each photostimulation. 
    newData{idx}=zeros(512,512,frameNum);                          % to store the final corrected recording after removing the artifact
    tempData1=data1;                                               % to store the temporal corrected recording after removing the artifact for horizontal scan
    tempData2=data1;                                               % to store the temporal corrected recording after removing the artifact for vertical scan
    for idx2=1:frameNum                                            % the following scan each frame during photostimulation
        for idx3=1:size(data0,2)                                   % scan 1: horizontal scan
            % horizontal scan
            temp=data1(:,idx3,idx2)-data0(:,idx3);                 % differeence between current pixel value and the "baseline" stored in data0.
            index=find(temp<artifactThr);
            index2=diff(index);
            index3=find(index2<=artifactWidth+1 & index2>1);       % find out the pixels that exceeds criteria set in "artifactThr" and are "connected" together. 
   
            for idx4=1:length(index3)
                tempData1((index(index3(idx4))+1):(index(index3(idx4)+1)-1),idx3,idx2)=(data1(index(index3(idx4)),idx3,idx2)+data1(index(index3(idx4)+1),idx3,idx2))/2;  % replace the contanminated pixels with their adjacent "clean" pixels.
            end
        end
           
        for idx3=1:size(data0,1)                                   % scan 2: vertical scan
            % vertical scan
            temp=data1(idx3,:,idx2)-data0(idx3,:);                 % differeence between current pixel value and the "baseline" stored in data0.
            index=find(temp<artifactThr);
            index2=diff(index);
            index3=find(index2<=artifactWidth+1 & index2>1);       % find out the pixels that exceeds criteria set in "artifactThr" and are "connected" together. 
   
            for idx4=1:length(index3)
                tempData2(idx3,(index(index3(idx4))+1):(index(index3(idx4)+1)-1),idx2)=(data1(idx3,index(index3(idx4)),idx2)+data1(idx3,index(index3(idx4)+1),idx2))/2;   % replace the contanminated pixels with their adjacents "clean" pixels.
            end
        end       
    end
    newData{idx}=uint16((tempData1+tempData2)/2);                  % average results obtains from the two scans.
% write the tiff stack for individual corrected photostimulation session
    nam=[saveFileName '_' num2str(idx) '.tif'];
    imwrite(newData{idx}(:,:,1), nam);
    for idx2 = 2:frameNum
        imwrite(newData{idx}(:,:,idx2), nam, 'writemode', 'append');
    end
end

