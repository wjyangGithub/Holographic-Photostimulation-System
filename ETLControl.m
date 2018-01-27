% Control the ETL for volumetric imaging
% This matlab code monitors the end-of-frame trigger from the image
% acquision system (e.g.Prairieview), and controls the ETL for volumetric
% imaging
% Author: Weijian Yang, 2015-2018

% Operation Principle (Imaging)
% At the end of each frame, Prairie sends out a trigger signal (PCI-6713 FTO) to NI box (PFI0). 
% Matlab receives this trigger and do a counting (with counter mode), until it reaches N. 
% Matlab then sends out a voltage signal through NI box (AO1) to the ETL/Piezo stage. 
% Note: the ETL input voltage should be from 0-5V.

% Connect AO1 from NI Box (output) to ETL/Piezo stage input (input), for ETL/Piezo control
% Connect PFI0 from NI Box to Prairie PCI-6713 FTO (output), this serves as a relay station. This is necessary for the PFI12 counter to work. 
% Connect PFI12 from NI Box (input) to Prairie PCI-6713 FIO (output) to count the end of frame trigger
% Note: for resonance scanning, Prairie PCI-6713 FIO is connected to "End of Frame Trigger Out" from the resonance scanning control
% Note: for resonance scanning, there can be a pulse shaping box (multi-vibrator) in between PFI12 from NI box and Prairie PCI-6713 FIO

% Set up "End of Frame Trigger" as "Output Trigger Type" in Prairie 
% Set up the "Frame Period" in Prairie
% Set up the "Trigger" and "Wait for Trigger" in spiral activation in Prairie as "TrigIn" and "EveryRepetition". 
% Set up the correct number of "Reps" in spiral activation in Prairie

% Setup AO1
session = daq.createSession ('ni');
addAnalogOutputChannel(session,'dev1',1,'Voltage');
outputSingleScan(session,[1.65]);    % output [AO1]

% Setup counter, PFI12
addCounterInputChannel(session,'dev1', 'ctr2', 'EdgeCount');
resetCounters(session);
inputSingleScan(session);

% Setup imaging parameters
% Mapping of the ETL
voltage=   [0   0.3 0.5  0.65  0.75  0.85  1.05  1.2  1.4   1.55  1.6   1.65  1.8  1.9  2    2.1   2.3  2.45  2.6  2.75  2.9   3.05  3.2  3.35  3.5 ];     % [V] voltage to thorlab current controller
focusShift=[90  76   72    61    58    52    40   32   19   10     5    0     -10  -18  -26  -34   -48  -64   -78  -90   -100  -117  -135 -150  -170 ];     % [um] for 25X fat objective. Positive means longer focal length; negative means shorter focal length

zPosition=[2.6 2.2 1.65 1.05 0.2];                 % the z position to be sampled: voltage; there would be a look up table soon 
zPositionNum=length(zPosition);       % total number of z positions to be sampled
dwellFrame=1;                         % number of frames to dwell on each z position, default: 1
cycle=6000;                           % number of volumes to acquire 

imageFrameNum=zPositionNum*dwellFrame*cycle     % total number of frames to acquire

for ii=1:cycle                                           % for number of volumes to acquire
    for i=1:zPositionNum                                 % for number of z positions at each volume
        while inputSingleScan(session)~=dwellFrame
        end
        outputSingleScan(session,[zPosition(i)]);      % z position changes at the end of the frame
        resetCounters(session);
    end
end

outputSingleScan(session,1.65);