This set of programs suppress/remove the photostimulation artifacts due to photostimulation.
Procedure:
1. Run Matlab code "photostimulationArtifactRemover.m". This code takes in the raw recording and outputs corrected recording for individual corrected photostimulation session where the photostimulation artifacts are suppressed. If there are N photostimulation sessions, it will output N tif files.
2. Open ImageJ, run macro program "ConcatenateStack_ImageJMacro.txt". This macro takes in the raw recording, and replaces the contanminated recording sessions with the corrected ones output from the Matlab program. 

Detailed comments are written in both Matlab and ImageJ macro program.

Author: Weijian Yang, 2015-2018.
