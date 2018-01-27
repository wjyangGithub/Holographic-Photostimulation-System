% Main control of the photostimulation system
% This software can perform cooridinate calibration, selecting target
% points in 3D, load and switch holographic phase pattern on SLM, etc.
% Author: Weijian Yang, 2015-2018

% This program assumes the SLM is from BNS (Meadowlark Optics) or Holoeye 
% To use this program, uncomment Line 114-118, 150-154, 560-564 in this file
% and uncomment Line 32 and 36 in "f_SLMActivation_Calibration.m".

function varargout = MainControl(varargin)
%MAINCONTROL M-file for MainControl.fig
%      MAINCONTROL, by itself, creates a new MAINCONTROL or raises the existing
%      singleton*.
%
%      H = MAINCONTROL returns the handle to a new MAINCONTROL or the handle to
%      the existing singleton*.
%
%      MAINCONTROL('Property','Value',...) creates a new MAINCONTROL using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to MainControl_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MAINCONTROL('CALLBACK') and MAINCONTROL('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MAINCONTROL.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MainControl

% Last Modified by GUIDE v2.5 22-May-2017 21:12:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MainControl_OpeningFcn, ...
                   'gui_OutputFcn',  @MainControl_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MainControl is made visible.
function MainControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for MainControl
handles.output = hObject;

% self defined parameters
handles.SLM_handles=[];   % SLM handles
handles.f_SLMFocusCalFun=[];                  % fitted function of focal depth vs. effective NA
handles.tActivationLaserSLM=[];               % lateral calibration matrix
handles.tActivationLaserSLM_focusPlane=[];    % lateral calibration focus plane
handles.tActivationLaserSLM_calImageSize=[];  % lateral calibration image size
handles.tImage2Activation=[];                 % two laser mapping calibration matrix
handles.calImageSize=[];                      % lateral calibration image size
handles.XYWeightCorrection=[];                % correction weight (E field) for XY
handles.ZWeightCorrection=[];                 % correction weight (E field) for Z
handles.imageCorrectionFactorX=1.00;          % Image scale correction factor
handles.imageCorrectionFactorY=1.00;          % Image scale correction factor
handles.extraCorrection=[];
handles.extraCorrectionCalFun1=[];
handles.extraCorrectionCalFun2=[];
handles.extraCorrectionCalFun3=[];
handles.extraCorrectionCalFun4=[];
handles.scanZoom=1;

CFG;
handles.SLMPreset=SLMPreset;

handles.imageStack=[];
handles.imageID=0;
handles.imageNum=[];
handles.imageStack_imageHandle=[];
handles.imageStackROI_labelHandle=[];
handles.imageStackROI_imageHandle=[];

handles.selectROITag=0;            % tag for "Add ROI" and "Delete ROI" button
handles.ROINum=0;
handles.ROIList=[];
handles.ROIWeight=[];
handles.zPosition=[];

handles.stackTag=0;

handles.SLMxyzp=[];                % xyzp coordinate sent to BNS SLM
handles.objectiveNA=[];            % objective NA sent to BNS SLM
handles.SLMPhase=[];               % SLM phase pattern, which can be directly applied to SLM

handles.clickTolerance=2;

% initialize SLM
CFG;
handles.SLM_handles=1;
% if strcmp(SLM, 'BNS')
%     handles.SLM_handles = BNS_SLM_Initialize(calibratedLUTfileName);
% else
%     handles.SLM_handles = Holoeye_SLM_LoadImage(zeros(SLMm, SLMn), SLM_M, SLM_N, SLMm0, SLMn0);
% end
weight=1;
objectiveNA=0.4;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], weight, objectiveNA );  

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MainControl wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MainControl_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

CFG;
% if strcmp(SLM, 'BNS')
%     BNS_SLM_Close(handles.SLM_handles);
% else
%     close(handles.SLM_handles);
% end

delete(hObject);


function CalibrationFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to CalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CalibrationFile_edit as text
%        str2double(get(hObject,'String')) returns contents of CalibrationFile_edit as a double


% --- Executes during object creation, after setting all properties.
function CalibrationFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseCalibrationFile_button.
function BrowseCalibrationFile_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseCalibrationFile_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.CalibrationFile_edit, 'String', [pathName fileName]);

load([pathName fileName]);
handles.f_SLMFocusCalFun=f_SLMFocusCalFun;
handles.tActivationLaserSLM=tActivationLaserSLM;               
handles.tActivationLaserSLM_focusPlane=tActivationLaserSLM_focusPlane;
handles.tActivationLaserSLM_calImageSize=tActivationLaserSLM_calImageSize;
handles.calImageSize=tActivationLaserSLM_calImageSize(1,:);

handles.tImage2Activation=tImage2Activation;
handles.tImage2Activation_focusPlane=tImage2Activation_focusPlane;

handles.XYWeightCorrection=XYWeightCorrection;
handles.ZWeightCorrection=ZWeightCorrection; 

guidata(hObject, handles);


% --- Executes on button press in NewCalibration_button.
function NewCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to NewCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationControl(handles.SLM_handles);


% --- Executes on button press in AddROI_radiobutton.
function AddROI_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to AddROI_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of AddROI_radiobutton
if get(hObject,'Value')==1
    handles.selectROITag=1;
    set(handles.DeleteROI_radiobutton,'Value',0);
else
    handles.selectROITag=0;
end
guidata(hObject, handles);


% --- Executes on button press in DeleteROI_radiobutton.
function DeleteROI_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteROI_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of DeleteROI_radiobutton
if get(hObject,'Value')==1
    handles.selectROITag=-1;
    set(handles.AddROI_radiobutton,'Value',0);
else
    handles.selectROITag=0;
end
guidata(hObject, handles);


% --- Executes on button press in SLMActivation_button.
function SLMActivation_button_Callback(hObject, eventdata, handles)
% hObject    handle to SLMActivation_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ROI coordinate conversion
xyImage=handles.ROIList(:,1:2);
zPosition=handles.zPosition*1e-6;

% scale the xyImage coordinate based on the image pixel size (e.g. whether it is 512x512 or 256x256 image size, etc)
experimentImageSize=size(handles.imageStack,1);
scaleFactor=experimentImageSize/handles.calImageSize(1);   
xyImage=xyImage/scaleFactor;

% get the scan zoom
handles.ScanZoom=str2double(get(handles.ScanZoom_edit, 'String'));

% Galvo center point to correct the target point on xy activation
% activation laser center point
activationCenter=handles.calImageSize/2;

% image laser center point, find the calibration file at 0 um plane
[dump, zPositionID]=min(abs(handles.tImage2Activation_focusPlane-0));
imageCenter=tformfwd(handles.tImage2Activation{zPositionID}(end), activationCenter);
%imageCenter=[186 262];     % for 25X resonant scanner
set(handles.StatusWindow_text,'String',['Please place galvo at x=' num2str((imageCenter(1)*scaleFactor)) '; y=' num2str((imageCenter(2)*scaleFactor))]);
uiwait(msgbox(['Please place galvo at x=' num2str((imageCenter(1)*scaleFactor)) '; y=' num2str((imageCenter(2)*scaleFactor)) '. When ready, press ENTER...']));

%% extra correction factor
if ~isempty(handles.extraCorrection)
    shiftX=handles.extraCorrectionCalFun1(zPosition*1e6);
    shiftY=handles.extraCorrectionCalFun2(zPosition*1e6);
    magX=handles.extraCorrectionCalFun3(zPosition*1e6);
    magY=handles.extraCorrectionCalFun4(zPosition*1e6);

    xyImage(:,1)=(xyImage(:,1)-imageCenter(1)*scaleFactor+shiftX).*magX+imageCenter(1)*scaleFactor;
    xyImage(:,2)=(xyImage(:,2)-imageCenter(2)*scaleFactor+shiftY).*magY+imageCenter(2)*scaleFactor;
end

%% correct for the scan zoom
xyImage(:,1)=(xyImage(:,1)-imageCenter(1)*scaleFactor)/handles.ScanZoom+imageCenter(1)*scaleFactor;
xyImage(:,2)=(xyImage(:,2)-imageCenter(2)*scaleFactor)/handles.ScanZoom+imageCenter(2)*scaleFactor;


%% Construct 25 point zone
% 25 point zone
[zoneX zoneY]=meshgrid(handles.calImageSize(1)-handles.calImageSize(1)/6:-handles.calImageSize(1)/6:handles.calImageSize(1)/6, ...
                       handles.calImageSize(1)/6:handles.calImageSize(1)/6:handles.calImageSize(1)-handles.calImageSize(1)/6);
zoneX=zoneX'; zoneX=zoneX(:);
zoneY=zoneY'; zoneY=zoneY(:);

%% Check whether imaging stack or activation stack is loaded
stackTag=get(handles.ActivationLaserStack_checkbox, 'Value');
handles.stackTag=stackTag;
if stackTag==1      % activation stack, no need to do xyImage to xyActivation transfer
    xyActivation=xyImage;
else                % image stack, need to do a transform from xy image to xy activation    
% the following consider different focus for tImage2Activation, use 'linearinterp' for fitting
    xyActivation=zeros(size(xyImage));
    for idx=1:size(xyImage,1)
        [dump, zPositionID]=min(abs(handles.tImage2Activation_focusPlane-zPosition(idx)));
        [dump, zoneID]=min((zoneX-xyImage(idx,1)).^2+(zoneY-xyImage(idx,2)).^2);
        
        if zPositionID==1
            tempzID=[1 2];
        else if zPositionID==length(handles.tImage2Activation_focusPlane)
                tempzID=[length(handles.tImage2Activation_focusPlane)-1 length(handles.tImage2Activation_focusPlane)];
            else
                tempzID=[zPositionID-1 zPositionID zPositionID+1];
            end
        end
            
        for idx1=1:length(tempzID)
            xyTemp(idx1,:)=tforminv(handles.tImage2Activation{tempzID(idx1)}(zoneID),xyImage(idx,:));
        end
        
        [calFun1,dump1,dump2] = fit( handles.tImage2Activation_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
        [calFun2,dump1,dump2] = fit( handles.tImage2Activation_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

        xyActivation(idx,1)=calFun1(zPosition(idx));
        xyActivation(idx,2)=calFun2(zPosition(idx));        
    end
end

%% correct xy activation because we use the laser for activation instead of imaging
xyActivationCorrected=xyActivation;
xyActivationCorrected(:,1)=handles.calImageSize(1)+1-xyActivation(:,1);
xyActivationCorrected(:,2)=handles.calImageSize(1)+1-xyActivation(:,2);

% transform from xy activation to xyp plane
xyp=zeros(size(xyActivationCorrected));
                   
% the following consider different focus for tActivationLaserSLM, use 'linearinterp' for fitting 
for idx=1:size(xyp,1)
    [dump, zPositionID]=min(abs(handles.tActivationLaserSLM_focusPlane-zPosition(idx)));
    [dump, zoneID]=min((zoneX-xyActivationCorrected(idx,1)).^2+(zoneY-xyActivationCorrected(idx,2)).^2);
    xyp(idx,:)=tforminv(handles.tActivationLaserSLM{zPositionID}(zoneID),xyActivationCorrected(idx,:));
    
    if zPositionID==1
        tempzID=[1 2];
    else if zPositionID==length(handles.tActivationLaserSLM_focusPlane)
            tempzID=[length(handles.tActivationLaserSLM_focusPlane)-1 length(handles.tActivationLaserSLM_focusPlane)];
        else
            tempzID=[zPositionID-1 zPositionID zPositionID+1];
        end
    end
            
    clear xyTemp;
    for idx1=1:length(tempzID)
        xyTemp(idx1,:)=tforminv(handles.tActivationLaserSLM{tempzID(idx1)}(zoneID),xyActivationCorrected(idx,:));
    end
        
    [calFun1,dump1,dump2] = fit( handles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
    [calFun2,dump1,dump2] = fit( handles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

    xyp(idx,1)=calFun1(zPosition(idx));
    xyp(idx,2)=calFun2(zPosition(idx));        
end

% add the z column
xyzp=zeros(size(xyp,1),3);
xyzp(:,1:2)=xyp;
xyzp(:,3)=handles.zPosition*1e-6;
objectiveNA=handles.f_SLMFocusCalFun(xyzp(:,3));
weight=handles.ROIWeight;

handles.SLMxyzp=xyzp;                
handles.objectiveNA=objectiveNA;

handles.SLMPhase=f_SLMActivation_Calibration( handles.SLM_handles, xyzp, weight, objectiveNA );

guidata(hObject,handles);


% --- Executes on button press in SLMReset_button.
function SLMReset_button_Callback(hObject, eventdata, handles)
% hObject    handle to SLMReset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
weight=1;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], weight, 0.4 );


function ImageFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ImageFile_edit as text
%        str2double(get(hObject,'String')) returns contents of ImageFile_edit as a double


% --- Executes during object creation, after setting all properties.
function ImageFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseImageFile_button.
function BrowseImageFile_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseImageFile_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% read image stack file
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ImageFile_edit, 'String', [pathName fileName]);
handles.imageStack=f_readTiffFile([pathName fileName]);

handles.imageID=1;
handles.imageNum=size(handles.imageStack,3);

% plot image file
axes(handles.ImageStack_axe);
handles.imageStack_imageHandle=imagesc(handles.imageStack(:,:,handles.imageID));
colormap('gray');
daspect([1 1 1]);

% update image ID display
set(handles.ImageID_text, 'String', [num2str(handles.imageID) ' of ' num2str(handles.imageNum)]);

% update slider
set(handles.FocalPlane_slider,'Min',1);
set(handles.FocalPlane_slider,'Max',max(handles.imageNum,1.01));
set(handles.FocalPlane_slider,'Value',max(handles.imageNum,1.01)+1-handles.imageID);
set(handles.FocalPlane_slider,'SliderStep',[1/handles.imageNum, 10/handles.imageNum]);

% setup buttondownfcn for shifted image
set(handles.imageStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);


% --- Executes on button press in SaveList_button.
function SaveList_button_Callback(hObject, eventdata, handles)
% hObject    handle to SaveList_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save ROI List');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. ROI List file cannot be saved', 'Error');
    return;
end
ROITableData = get(handles.ROI_table,'Data');
SLMPhase = handles.SLMPhase;
save([path file],'ROITableData','SLMPhase');

% --- Executes on button press in LoadList_button.
function LoadList_button_Callback(hObject, eventdata, handles)
% hObject    handle to LoadList_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No ROI list file is selected.','error');
    return;
end
load([pathName fileName]);
set(handles.ROI_table,'Data',ROITableData);
handles.ROIList=ROITableData(:,1:3);
handles.zPosition=ROITableData(:,4);
handles.ROIWeight=ROITableData(:,5);

% updata graph
% delete the ROI points image handle
if ~isempty(handles.imageStackROI_imageHandle)
    delete(handles.imageStackROI_imageHandle);
    delete(handles.imageStackROI_labelHandle);
    handles.imageStackROI_imageHandle=[];
    handles.imageStackROI_labelHandle=[];
end

index=find(handles.ROIList(:,3)==handles.imageID);
if ~isempty(index)
% replot the ROI points image
     axes(handles.ImageStack_axe);
%     figure(handles.imageStack_imageHandle);
     hold on;
     handles.imageStackROI_imageHandle=plot(handles.ROIList(index,1),handles.ROIList(index,2),'ro');
     handles.imageStackROI_labelHandle=text(handles.ROIList(index,1)+3,handles.ROIList(index,2)-3,num2str(index),'color',[1 0 0],'fontsize',10);
     set(handles.imageStackROI_imageHandle, 'HitTest','off');
     set(handles.imageStackROI_labelHandle, 'HitTest','off');                     
end
set(handles.StatusWindow2_text,'String',['Total Weight=' num2str(sum(handles.ROIWeight)) '; Equivalent Point Num = ' num2str(sum(handles.ROIWeight.^2))]);
guidata(hObject, handles);


function ScanZoom_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ScanZoom_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ScanZoom_edit as text
%        str2double(get(hObject,'String')) returns contents of ScanZoom_edit as a double


% --- Executes during object creation, after setting all properties.
function ScanZoom_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ScanZoom_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ObjectiveLens_popupmenu.
function ObjectiveLens_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to ObjectiveLens_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ObjectiveLens_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ObjectiveLens_popupmenu


% --- Executes during object creation, after setting all properties.
function ObjectiveLens_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ObjectiveLens_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CFG;
% if strcmp(SLM, 'BNS')
%     BNS_SLM_Close(handles.SLM_handles);
% else
%     close(handles.SLM_handles);
% end

hf=findobj('Name','MainControl');
delete(hf);

% --- Executes on slider movement.
function FocalPlane_slider_Callback(hObject, eventdata, handles)
% hObject    handle to FocalPlane_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% check slider position
sliderValue=handles.imageNum+1-round(get(hObject,'Value'));
if handles.imageID==sliderValue
    return;
end
handles.imageID=sliderValue;

% plot image stack file
axes(handles.ImageStack_axe);
handles.imageStack_imageHandle=imagesc(handles.imageStack(:,:,handles.imageID));
colormap('gray');
daspect([1 1 1]);

% update image ID display
set(handles.ImageID_text, 'String', [num2str(handles.imageID) ' of ' num2str(handles.imageNum)]);
set(handles.FocalPlane_edit, 'String', str2double(get(handles.FirstPlanePosition_edit,'String'))+(handles.imageID-1)*str2double(get(handles.FocalPlane_umperframe_edit,'String')));

% setup buttondownfcn for image stack
set(handles.imageStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% plot the ROI points
% delete the ROI points image handle
if ~isempty(handles.imageStackROI_imageHandle)
    delete(handles.imageStackROI_imageHandle);
    delete(handles.imageStackROI_labelHandle);
    handles.imageStackROI_imageHandle=[];
    handles.imageStackROI_labelHandle=[];
end
% replot the ROI points image
hold on;
if ~isempty(handles.ROIList)
    index=find(handles.ROIList(:,3)==handles.imageID);
    if ~isempty(index)
        handles.imageStackROI_imageHandle=plot(handles.ROIList(index,1),handles.ROIList(index,2),'ro');
        handles.imageStackROI_labelHandle=text(handles.ROIList(index,1)+3,handles.ROIList(index,2)-3,num2str(index),'color',[1 0 0],'fontsize',10);
        set(handles.imageStackROI_imageHandle, 'HitTest','off');
        set(handles.imageStackROI_labelHandle, 'HitTest','off');                     
    end
end
   
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function FocalPlane_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FocalPlane_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function FocalPlane_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FocalPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox('close');

% Hints: get(hObject,'String') returns contents of FocalPlane_edit as text
%        str2double(get(hObject,'String')) returns contents of FocalPlane_edit as a double


% --- Executes during object creation, after setting all properties.
function FocalPlane_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FocalPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FocalPlane_umperframe_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FocalPlane_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FocalPlane_umperframe_edit as text
%        str2double(get(hObject,'String')) returns contents of FocalPlane_umperframe_edit as a double


% --- Executes during object creation, after setting all properties.
function FocalPlane_umperframe_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FocalPlane_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FirstPlanePosition_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FirstPlanePosition_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FirstPlanePosition_edit as text
%        str2double(get(hObject,'String')) returns contents of FirstPlanePosition_edit as a double


% --- Executes during object creation, after setting all properties.
function FirstPlanePosition_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FirstPlanePosition_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ImageClickCallback ( objectHandle , eventData, hObject, handles )
   handles = guidata(hObject);

% obtain coordinate
   axesHandle  = get(objectHandle,'Parent');
   coordinates = get(axesHandle,'CurrentPoint'); 
   x = coordinates(1,1);
   y = coordinates(1,2);

   if objectHandle==handles.imageStack_imageHandle     % image stack
       if handles.selectROITag==1         % add points
           handles.ROINum=handles.ROINum+1;
           handles.ROIList=[handles.ROIList; [x y handles.imageID]];
           handles.ROIWeight=[handles.ROIWeight; 1];
           handles.zPosition=[handles.zPosition; str2double(get(handles.FirstPlanePosition_edit,'String'))+(handles.imageID-1)*str2double(get(handles.FocalPlane_umperframe_edit,'String'))];           
           % delete the ROI points image handle
           if ~isempty(handles.imageStackROI_imageHandle)
               delete(handles.imageStackROI_imageHandle);
               delete(handles.imageStackROI_labelHandle);
           end
           % replot the ROI points image
           hold on;
           index=find(handles.ROIList(:,3)==handles.imageID);
           handles.imageStackROI_imageHandle=plot(handles.ROIList(index,1),handles.ROIList(index,2),'ro');
           handles.imageStackROI_labelHandle=text(handles.ROIList(index,1)+3,handles.ROIList(index,2)-3,num2str(index),'color',[1 0 0],'fontsize',10);
           set(handles.imageStackROI_imageHandle, 'HitTest','off');
           set(handles.imageStackROI_labelHandle, 'HitTest','off'); 
       else if handles.selectROITag==-1 & ~isempty(handles.ROIList)  % delete points
           index=find( sqrt((handles.ROIList(:,1)-x).^2+(handles.ROIList(:,2)-y).^2) < handles.clickTolerance & handles.ROIList(:,3)==handles.imageID );  
           if ~isempty(index)
               handles.ROINum=handles.ROINum-length(index);
               handles.ROIList(index,:)=[];
               handles.ROIWeight(index)=[];
               handles.zPosition(index)=[];
           % delete the ROI points image handle
               delete(handles.imageStackROI_imageHandle);
               delete(handles.imageStackROI_labelHandle);
               handles.imageStackROI_imageHandle=[];
               handles.imageStackROI_labelHandle=[];
               index=find(handles.ROIList(:,3)==handles.imageID);
               if ~isempty(index)
            % replot the ROI points image
                  hold on;
                  handles.imageStackROI_imageHandle=plot(handles.ROIList(index,1),handles.ROIList(index,2),'ro');
                  handles.imageStackROI_labelHandle=text(handles.ROIList(index,1)+3,handles.ROIList(index,2)-3,num2str(index),'color',[1 0 0],'fontsize',10);
                  set(handles.imageStackROI_imageHandle, 'HitTest','off');
                  set(handles.imageStackROI_labelHandle, 'HitTest','off');
               end
           end
           end
       end
  % update ROI table
     set(handles.ROI_table,'Data',[handles.ROIList handles.zPosition handles.ROIWeight]);
     set(handles.StatusWindow2_text,'String',['Total Weight=' num2str(sum(handles.ROIWeight)) '; Equivalent Point Num = ' num2str(sum(handles.ROIWeight.^2))]);   end
guidata(hObject, handles);


% --- Executes when entered data in editable cell(s) in ROI_table.
function ROI_table_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to ROI_table (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.ROIList)
    return;
end

% change weight
if eventdata.Indices(2)==5 && ~isempty(eventdata.NewData)
    handles.ROIWeight(eventdata.Indices(1))=eventdata.NewData;
end

% change x position
if eventdata.Indices(2)==1 && ~isempty(eventdata.NewData)
    handles.ROIList(eventdata.Indices(1),1)=eventdata.NewData;
end

% change y position
if eventdata.Indices(2)==2 && ~isempty(eventdata.NewData)
    handles.ROIList(eventdata.Indices(1),2)=eventdata.NewData;
end

% change z plane ID
if eventdata.Indices(2)==3 && ~isempty(eventdata.NewData)
    handles.ROIList(eventdata.Indices(1),3)=eventdata.NewData;
end

% change z position
if eventdata.Indices(2)==4 && ~isempty(eventdata.NewData)
    handles.zPosition(eventdata.Indices(1))=eventdata.NewData;
end
    
% updata graph
% delete the ROI points image handle
delete(handles.imageStackROI_imageHandle);
delete(handles.imageStackROI_labelHandle);
handles.imageStackROI_imageHandle=[];
handles.imageStackROI_labelHandle=[];

index=find(handles.ROIList(:,3)==handles.imageID);
if ~isempty(index)
% replot the ROI points image
     axes(handles.ImageStack_axe);
%     figure(handles.imageStack_imageHandle);
     hold on;
     handles.imageStackROI_imageHandle=plot(handles.ROIList(index,1),handles.ROIList(index,2),'ro');
     handles.imageStackROI_labelHandle=text(handles.ROIList(index,1)+3,handles.ROIList(index,2)-3,num2str(index),'color',[1 0 0],'fontsize',10);
     set(handles.imageStackROI_imageHandle, 'HitTest','off');
     set(handles.imageStackROI_labelHandle, 'HitTest','off');                     
end
set(handles.StatusWindow2_text,'String',['Total Weight=' num2str(sum(handles.ROIWeight)) '; Equivalent Point Num = ' num2str(sum(handles.ROIWeight.^2))]);
guidata(hObject, handles);


% --- Executes on button press in ActivationLaserStack_checkbox.
function ActivationLaserStack_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationLaserStack_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ActivationLaserStack_checkbox


% --- Executes on button press in SaveImage_button.
function SaveImage_button_Callback(hObject, eventdata, handles)
% hObject    handle to SaveImage_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.fig','Save Image');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Image file cannot be saved', 'Error');
    return;
end

saveas(handles.ImageStack_axe, [path file]);


% --- Executes on button press in PatternSwitch_button.
function PatternSwitch_button_Callback(hObject, eventdata, handles)
% hObject    handle to PatternSwitch_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PatternSwitch(handles.SLM_handles);


% --- Executes on button press in CalibrationTest_button.
function CalibrationTest_button_Callback(hObject, eventdata, handles)
% hObject    handle to CalibrationTest_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationTest(handles);


% --- Executes on button press in AutoWeight_button.
function AutoWeight_button_Callback(hObject, eventdata, handles)
% hObject    handle to AutoWeight_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% this code is the same as the callback function of "SLM Activation"
% convert the ROI coordinate into the SLM coordinate

% ROI coordinate conversion
xyImage=handles.ROIList(:,1:2);
zPosition=handles.zPosition*1e-6;

% scale the xyImage coordinate based on the image pixel size (e.g. whether it is 512x512 or 256x256 image size, etc)
experimentImageSize=size(handles.imageStack,1);
scaleFactor=experimentImageSize/handles.calImageSize(1);   
xyImage=xyImage/scaleFactor;

% get the scan zoom
handles.ScanZoom=str2double(get(handles.ScanZoom_edit, 'String'));

% Galvo center point to correct the target point on xy activation
% activation laser center point
activationCenter=handles.calImageSize/2;

% image laser center point, find the calibration file at 0 um plane
[dump, zPositionID]=min(abs(handles.tImage2Activation_focusPlane-0));
imageCenter=tformfwd(handles.tImage2Activation{zPositionID}(end), activationCenter);

% extra correction factor
if ~isempty(handles.extraCorrection)
    shiftX=handles.extraCorrectionCalFun1(zPosition*1e6);
    shiftY=handles.extraCorrectionCalFun2(zPosition*1e6);
    magX=handles.extraCorrectionCalFun3(zPosition*1e6);
    magY=handles.extraCorrectionCalFun4(zPosition*1e6);

    xyImage(:,1)=(xyImage(:,1)-imageCenter(1)*scaleFactor+shiftX).*magX+imageCenter(1)*scaleFactor;
    xyImage(:,2)=(xyImage(:,2)-imageCenter(2)*scaleFactor+shiftY).*magY+imageCenter(2)*scaleFactor;
end

% correct for the scan zoom
xyImage(:,1)=(xyImage(:,1)-imageCenter(1)*scaleFactor)/handles.ScanZoom+imageCenter(1)*scaleFactor;
xyImage(:,2)=(xyImage(:,2)-imageCenter(2)*scaleFactor)/handles.ScanZoom+imageCenter(2)*scaleFactor;

%% Construct 25 point zone
% 25 point zone
[zoneX zoneY]=meshgrid(handles.calImageSize(1)-handles.calImageSize(1)/6:-handles.calImageSize(1)/6:handles.calImageSize(1)/6, ...
                       handles.calImageSize(1)/6:handles.calImageSize(1)/6:handles.calImageSize(1)-handles.calImageSize(1)/6);
zoneX=zoneX'; zoneX=zoneX(:);
zoneY=zoneY'; zoneY=zoneY(:);

%% Check whether imaging stack or activation stack is loaded
stackTag=get(handles.ActivationLaserStack_checkbox, 'Value');
handles.stackTag=stackTag;
if stackTag==1      % activation stack, no need to do xyImage to xyActivation transfer
    xyActivation=xyImage;
else                % image stack, need to do a transform from xy image to xy activation    
% the following consider different focus for tImage2Activation, use 'linearinterp' for fitting
    xyActivation=zeros(size(xyImage));
    for idx=1:size(xyImage,1)
        [dump, zPositionID]=min(abs(handles.tImage2Activation_focusPlane-zPosition(idx)));
        [dump, zoneID]=min((zoneX-xyImage(idx,1)).^2+(zoneY-xyImage(idx,2)).^2);
        
        if zPositionID==1
            tempzID=[1 2];
        else if zPositionID==length(handles.tImage2Activation_focusPlane)
                tempzID=[length(handles.tImage2Activation_focusPlane)-1 length(handles.tImage2Activation_focusPlane)];
            else
                tempzID=[zPositionID-1 zPositionID zPositionID+1];
            end
        end
            
        for idx1=1:length(tempzID)
            xyTemp(idx1,:)=tforminv(handles.tImage2Activation{tempzID(idx1)}(zoneID),xyImage(idx,:));
        end
        
        [calFun1,dump1,dump2] = fit( handles.tImage2Activation_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
        [calFun2,dump1,dump2] = fit( handles.tImage2Activation_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

        xyActivation(idx,1)=calFun1(zPosition(idx));
        xyActivation(idx,2)=calFun2(zPosition(idx));        
    end
end

%% correct xy activation because we use the laser for activation instead of imaging
xyActivationCorrected=xyActivation;
xyActivationCorrected(:,1)=handles.calImageSize(1)+1-xyActivation(:,1);
xyActivationCorrected(:,2)=handles.calImageSize(1)+1-xyActivation(:,2);

% transform from xy activation to xyp plane
xyp=zeros(size(xyActivationCorrected));
                   
% the following consider different focus for tActivationLaserSLM, use 'linearinterp' for fitting 
for idx=1:size(xyp,1)
    [dump, zPositionID]=min(abs(handles.tActivationLaserSLM_focusPlane-zPosition(idx)));
    [dump, zoneID]=min((zoneX-xyActivationCorrected(idx,1)).^2+(zoneY-xyActivationCorrected(idx,2)).^2);
    xyp(idx,:)=tforminv(handles.tActivationLaserSLM{zPositionID}(zoneID),xyActivationCorrected(idx,:));
    
    if zPositionID==1
        tempzID=[1 2];
    else if zPositionID==length(handles.tActivationLaserSLM_focusPlane)
            tempzID=[length(handles.tActivationLaserSLM_focusPlane)-1 length(handles.tActivationLaserSLM_focusPlane)];
        else
            tempzID=[zPositionID-1 zPositionID zPositionID+1];
        end
    end
            
    clear xyTemp;
    for idx1=1:length(tempzID)
        xyTemp(idx1,:)=tforminv(handles.tActivationLaserSLM{tempzID(idx1)}(zoneID),xyActivationCorrected(idx,:));
    end
        
    [calFun1,dump1,dump2] = fit( handles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
    [calFun2,dump1,dump2] = fit( handles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

    xyp(idx,1)=calFun1(zPosition(idx));
    xyp(idx,2)=calFun2(zPosition(idx));        
end

% add the z column
xyzp=zeros(size(xyp,1),3);
xyzp(:,1:2)=xyp;
xyzp(:,3)=handles.zPosition*1e-6;

%% apply the auto weight for xy
weight=[];
if ~isempty(handles.XYWeightCorrection)
    weightXY=interp2(handles.XYWeightCorrection.X,handles.XYWeightCorrection.Y,handles.XYWeightCorrection.Mat,xyzp(:,1),xyzp(:,2));
    weight=abs(weightXY);
end

%% apply the auto weight for z
if ~isempty(handles.ZWeightCorrection)
    weightZ=interp1(handles.ZWeightCorrection.Z,handles.ZWeightCorrection.Mat,xyzp(:,3));
    if isempty(weight)
        weight=abs(weightZ);
    else
        weight=weight.*abs(weightZ);
    end
end

%% update ROI table
if ~isempty(weight)
    weight=weight/min(weight);
    handles.ROIWeight=weight;
    set(handles.ROI_table,'Data',[handles.ROIList handles.zPosition handles.ROIWeight]);  
    set(handles.StatusWindow2_text,'String',['Total Weight=' num2str(sum(handles.ROIWeight)) '; Equivalent Point Num = ' num2str(sum(handles.ROIWeight.^2))]);
end
guidata(hObject,handles);


% --- Executes on button press in ExtraCorrection_button.
function ExtraCorrection_button_Callback(hObject, eventdata, handles)
% hObject    handle to ExtraCorrection_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.txt');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No ExtraCorrection file is selected.','error');
    return;
end
handles.extraCorrection=load([pathName fileName]);
handles.extraCorrectionCalFun1=fit(handles.extraCorrection(:,1),handles.extraCorrection(:,2),'linearinterp');
handles.extraCorrectionCalFun2=fit(handles.extraCorrection(:,1),handles.extraCorrection(:,3),'linearinterp');
handles.extraCorrectionCalFun3=fit(handles.extraCorrection(:,1),handles.extraCorrection(:,4),'linearinterp');
handles.extraCorrectionCalFun4=fit(handles.extraCorrection(:,1),handles.extraCorrection(:,5),'linearinterp');

guidata(hObject,handles);


