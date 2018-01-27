% Coordinate calibration between imaging laser and photostimulation laser: 
% to calibrate the coordinate between imaging laser and photostimulation
% laser in the imaging plane
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationControl_TwoLaserMapping(varargin)
%CALIBRATIONCONTROL_TWOLASERMAPPING M-file for CalibrationControl_TwoLaserMapping.fig
%      CALIBRATIONCONTROL_TWOLASERMAPPING, by itself, creates a new CALIBRATIONCONTROL_TWOLASERMAPPING or raises the existing
%      singleton*.
%
%      H = CALIBRATIONCONTROL_TWOLASERMAPPING returns the handle to a new CALIBRATIONCONTROL_TWOLASERMAPPING or the handle to
%      the existing singleton*.
%
%      CALIBRATIONCONTROL_TWOLASERMAPPING('Property','Value',...) creates a new CALIBRATIONCONTROL_TWOLASERMAPPING using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to CalibrationControl_TwoLaserMapping_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CALIBRATIONCONTROL_TWOLASERMAPPING('CALLBACK') and CALIBRATIONCONTROL_TWOLASERMAPPING('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CALIBRATIONCONTROL_TWOLASERMAPPING.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationControl_TwoLaserMapping

% Last Modified by GUIDE v2.5 04-Dec-2015 11:04:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationControl_TwoLaserMapping_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationControl_TwoLaserMapping_OutputFcn, ...
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


% --- Executes just before CalibrationControl_TwoLaserMapping is made visible.
function CalibrationControl_TwoLaserMapping_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for CalibrationControl_TwoLaserMapping
handles.output = hObject;

% Self defined parameters

handles.SLM_handles=varargin{1};    % SLM handle
CFG;
handles.SLMPreset=SLMPreset;
    
handles.imageSize=[];
handles.imageStack=[];
handles.imageStackID=0;
handles.imageStackNum=[];
handles.imageStack_imageHandle=[];

handles.activationSize=[];
handles.activationStack=[];
handles.activationStackID=0;
handles.activationStackNum=[];
handles.activationStack_imageHandle=[];

handles.calibrationTag=0;
handles.imageRotation=180;
handles.clickTolerance=2;
handles.imageStackSelectPointsTag=0;
handles.imageStackTarget_imageHandle=[];
handles.imageStackTarget_labelHandle=[];
handles.imageStackTargetNum=0;
handles.imageStackTargetList=[];

handles.activationStackSelectPointsTag=0;
handles.activationStackTarget_imageHandle=[];
handles.activationStackTarget_labelHandle=[];
handles.activationStackTargetNum=0;
handles.activationStackTargetList=[];

handles.currentFocus=0;

handles.imageStackTargetID=0;
handles.activationStackTargetID=0;
handles.targetNum=36;

handles.f_SLMFocusCalFun=[];
handles.tImage2Activation=[];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationControl_TwoLaserMapping wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationControl_TwoLaserMapping_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function ImageStackFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ImageStackFile_edit as text
%        str2double(get(hObject,'String')) returns contents of ImageStackFile_edit as a double


% --- Executes during object creation, after setting all properties.
function ImageStackFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageStackFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ActivationStackFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationStackFile_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationStackFile_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationStackFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStackFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseActivationStack_button.
function BrowseActivationStack_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseActivationStack_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ActivationStackFile_edit, 'String', [pathName fileName]);
handles.activationStack=f_readTiffFile([pathName fileName]);
handles.activationSize=[size(handles.activationStack,1) size(handles.activationStack,2)];

handles.activationStackID=1;
handles.activationStackNum=size(handles.activationStack,3);

% update activation stack rotation
handles.imageRotation=str2double(get(handles.ActivationLaserImageRotation_edit,'String'));

% plot activation stack file
axes(handles.ActivationStack_axe);
handles.activationStack_imageHandle=imagesc(imrotate(handles.activationStack(:,:,handles.activationStackID),handles.imageRotation));
colormap('gray');
daspect([1 1 1]);

% update activation ID display
set(handles.ActivationStackID_text, 'String', [num2str(handles.activationStackID) ' of ' num2str(handles.activationStackNum)]);

% update slider for activation stack
set(handles.ActivationStackFocus_slider,'Min',1);
set(handles.ActivationStackFocus_slider,'Max',max(handles.activationStackNum,1.01));
set(handles.ActivationStackFocus_slider,'Value',handles.activationStackID);
set(handles.ActivationStackFocus_slider,'SliderStep',[1/handles.activationStackNum, 10/handles.activationStackNum]);

% setup buttondownfcn for activation stack
set(handles.activationStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);



% --- Executes on button press in BrowseImageStack_button.
function BrowseImageStack_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseImageStack_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% read image stack file
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ImageStackFile_edit, 'String', [pathName fileName]);
handles.imageStack=f_readTiffFile([pathName fileName]);
handles.imageSize=[size(handles.imageStack,1) size(handles.imageStack,2)];

handles.imageStackID=1;
handles.imageStackNum=size(handles.imageStack,3);

% plot image stack file
axes(handles.ImageStack_axe);
handles.imageStack_imageHandle=imagesc(handles.imageStack(:,:,handles.imageStackID));
colormap('gray');
daspect([1 1 1]);

% update image ID display
set(handles.ImageStackID_text, 'String', [num2str(handles.imageStackID) ' of ' num2str(handles.imageStackNum)]);

% update slider for image stack
set(handles.ImageStackFocus_slider,'Min',1);
set(handles.ImageStackFocus_slider,'Max',max(handles.imageStackNum,1.01));
set(handles.ImageStackFocus_slider,'Value',handles.imageStackID);
set(handles.ImageStackFocus_slider,'SliderStep',[1/handles.imageStackNum, 10/handles.imageStackNum]);

% setup buttondownfcn for image stack
set(handles.imageStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);

% --- Executes on button press in ImageStackSelectPoints_radiobutton.
function ImageStackSelectPoints_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackSelectPoints_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ImageStackSelectPoints_radiobutton
if get(hObject,'Value')==1
    handles.imageStackSelectPointsTag=1;
    set(handles.ImageStackDeletePoints_radiobutton,'Value',0);
else
    handles.imageStackSelectPointsTag=0;
end
guidata(hObject, handles);

% --- Executes on button press in ImageStackDeletePoints_radiobutton.
function ImageStackDeletePoints_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackDeletePoints_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ImageStackDeletePoints_radiobutton
if get(hObject,'Value')==1
    handles.imageStackSelectPointsTag=-1;
    set(handles.ImageStackSelectPoints_radiobutton,'Value',0);
else
    handles.imageStackSelectPointsTag=0;
end
guidata(hObject, handles);

% --- Executes on button press in ActivationStackDeletePoints_radiobutton.
function ActivationStackDeletePoints_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackDeletePoints_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ActivationStackDeletePoints_radiobutton
if get(hObject,'Value')==1
    handles.activationStackSelectPointsTag=-1;
    set(handles.ActivationStackSelectPoints_radiobutton,'Value',0);
else
    handles.activationStackSelectPointsTag=0;
end
guidata(hObject, handles);

% --- Executes on button press in ActivationStackSelectPoints_radiobutton.
function ActivationStackSelectPoints_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackSelectPoints_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ActivationStackSelectPoints_radiobutton
if get(hObject,'Value')==1
    handles.activationStackSelectPointsTag=1;
    set(handles.ActivationStackDeletePoints_radiobutton,'Value',0);
else
    handles.activationStackSelectPointsTag=0;
end
guidata(hObject, handles);

% --- Executes on button press in StartAcqusition.
function StartAcqusition_Callback(hObject, eventdata, handles)
% hObject    handle to StartAcqusition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

startPlane=str2double(get(handles.SLMStartPlane_edit, 'String'));
endPlane=str2double(get(handles.SLMEndPlane_edit, 'String'));
planeInterval=str2double(get(handles.PlaneInterval_edit, 'String'));

if isempty(startPlane:planeInterval:endPlane)
    msgbox('Sign of Plane Interval Reversed', 'Error');
    planeInterval=-planeInterval;
    set(handles.PlaneInterval_edit, 'String',num2str(planeInterval));
end

planeStack=(startPlane:planeInterval:endPlane)*1e-6;
if isempty(handles.f_SLMFocusCalFun)
    objectiveNA=ones(1,length(planeStack))*0.4;
else
    objectiveNA=handles.f_SLMFocusCalFun(planeStack);
end

weight=[1];
framePeriod=str2double(get(handles.FramePeriod_edit,'String'));

% automatic acquision, start NI instrument and actuate SLM
session = daq.createSession ('ni');
session.addDigitalChannel('dev1','Port0/Line0','OutputOnly');
waitfor(msgbox(['Please set up Prairie to acquire ' num2str(length(planeStack)) ' images, and connect D0 port in NI box to Prairie input trigger. Press OK when ready.']));
for idx=1:length(planeStack)
    f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset planeStack(idx)], weight, objectiveNA(idx) );     
    session.outputSingleScan(1);
    pause(0.1);
    session.outputSingleScan(0);
    set(handles.AcquisitionID_text, 'String', [num2str(idx) ' of ' num2str(length(planeStack))]);
    pause(framePeriod);
end

weight=1;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], weight, objectiveNA(end) );
guidata(hObject,handles);


function PlaneInterval_edit_Callback(hObject, eventdata, handles)
% hObject    handle to PlaneInterval_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PlaneInterval_edit as text
%        str2double(get(hObject,'String')) returns contents of PlaneInterval_edit as a double


% --- Executes during object creation, after setting all properties.
function PlaneInterval_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PlaneInterval_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SLMEndPlane_edit_Callback(hObject, eventdata, handles)
% hObject    handle to SLMEndPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SLMEndPlane_edit as text
%        str2double(get(hObject,'String')) returns contents of SLMEndPlane_edit as a double


% --- Executes during object creation, after setting all properties.
function SLMEndPlane_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SLMEndPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SLMStartPlane_edit_Callback(hObject, eventdata, handles)
% hObject    handle to SLMStartPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SLMStartPlane_edit as text
%        str2double(get(hObject,'String')) returns contents of SLMStartPlane_edit as a double


% --- Executes during object creation, after setting all properties.
function SLMStartPlane_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SLMStartPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in StartCalibration_button.
function StartCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to StartCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% reset image stack ID and update display
handles.imageStackID=1;
set(handles.ImageStackFocus_slider,'Value',1);
set(handles.ImageStackID_text, 'String', [num2str(handles.imageStackID) ' of ' num2str(handles.imageStackNum)]);

% reset activation stack ID and update display
handles.activationStackID=1;
set(handles.ActivationStackFocus_slider,'Value',1);
set(handles.ActivationStackID_text, 'String', [num2str(handles.activationStackID) ' of ' num2str(handles.activationStackNum)]);

% plot image stack file
axes(handles.ImageStack_axe);
handles.imageStack_imageHandle=imagesc(handles.imageStack(:,:,handles.imageStackID));
colormap('gray');
daspect([1 1 1]);

% setup buttondownfcn for image stack
set(handles.imageStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% plot activation stack file
axes(handles.ActivationStack_axe);
handles.activationStack_imageHandle=imagesc(imrotate(handles.activationStack(:,:,handles.activationStackID),handles.imageRotation));

colormap('gray');
daspect([1 1 1]);

% setup buttondownfcn for activation stack
set(handles.activationStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% set indicators for calibration
handles.calibrationTag=1;
handles.imageStackSelectPointsTag=0;
handles.activationStackSelectPointsTag=0;

set(handles.ImageStackSelectPoints_radiobutton, 'Value',0);
set(handles.ImageStackDeletePoints_radiobutton, 'Value',0);
set(handles.ActivationStackSelectPoints_radiobutton, 'Value',0);
set(handles.ActivationStackDeletePoints_radiobutton, 'Value',0);

handles.imageStackTargetID=1;
handles.activationStackTargetID=1;
set(handles.ImageStackStatus_text, 'String', [num2str(handles.imageStackTargetID) ' of ' num2str(handles.targetNum)]);
set(handles.ActivationStackStatus_text, 'String', [num2str(handles.activationStackTargetID) ' of ' num2str(handles.targetNum)]);

guidata(hObject, handles);


% --- Executes on button press in FinishCalibration_button.
function FinishCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to FinishCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.imageRotation==180
    handles.activationStackTargetList(:,1)=size(handles.activationStack,2)+1-handles.activationStackTargetList(:,1);
    handles.activationStackTargetList(:,2)=size(handles.activationStack,1)+1-handles.activationStackTargetList(:,2);
end

% use all the point to construct transfer matrix
%handles.tImage2Activation=cp2tform(handles.activationStackTargetList, handles.imageStackTargetList, 'polynomial', 4);
handles.tImage2Activation=cp2tform(handles.activationStackTargetList, handles.imageStackTargetList, 'affine');

% construct the 25 zones
zone=[1 2 7 8]; 
for idx=1:25
    zoneIndex=zone+(idx-1)+floor((idx-1)/5);
    tempT(idx)=cp2tform(handles.activationStackTargetList(zoneIndex,:), handles.imageStackTargetList(zoneIndex,:), 'affine');
end
 
tempT(idx+1)=handles.tImage2Activation;
handles.tImage2Activation=tempT; 

guidata(hObject, handles);


% --- Executes on button press in SaveCalibration_button.
function SaveCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to SaveCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save Lateral Calibration File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration file cannot be saved', 'Error');
    return;
end
tImage2Activation=handles.tImage2Activation;
focusPlane=str2double(get(handles.CurrentFocus_edit,'String'));
focusPlane=focusPlane/1e6;
activationStackTargetList=handles.activationStackTargetList;
imageStackTargetList=handles.imageStackTargetList;
imageSize=handles.imageSize;
activationSize=handles.activationSize;

save([path file],'tImage2Activation','focusPlane','activationStackTargetList','imageStackTargetList','imageSize','activationSize');
msgbox('Calibration file Saved. If Current Focus Plane has not yet been updated, please update and save again.','Confirm');


% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hf=findobj('Name','CalibrationControl_TwoLaserMapping');
delete(hf);


function ActivationStackFocus_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackFocus_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationStackFocus_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationStackFocus_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationStackFocus_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStackFocus_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function ActivationStackFocus_slider_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackFocus_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% check slider position
sliderValue=round(get(hObject,'Value'));
if handles.activationStackID==sliderValue
    return;
end
handles.activationStackID=sliderValue;

% plot activation stack file
axes(handles.ActivationStack_axe);
handles.activationStack_imageHandle=imagesc(imrotate(handles.activationStack(:,:,handles.activationStackID),handles.imageRotation));

colormap('gray');
daspect([1 1 1]);

% update activation ID display
set(handles.ActivationStackID_text, 'String', [num2str(handles.activationStackID) ' of ' num2str(handles.activationStackNum)]);

% setup buttondownfcn for activation stack
set(handles.activationStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ActivationStackFocus_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStackFocus_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function ActivationStack_umperframe_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStack_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationStack_umperframe_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationStack_umperframe_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationStack_umperframe_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStack_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CurrentFocus_edit_Callback(hObject, eventdata, handles)
% hObject    handle to CurrentFocus_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CurrentFocus_edit as text
%        str2double(get(hObject,'String')) returns contents of CurrentFocus_edit as a double


% --- Executes during object creation, after setting all properties.
function CurrentFocus_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CurrentFocus_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function ImageStackFocus_slider_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackFocus_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% check slider position
sliderValue=round(get(hObject,'Value'));
if handles.imageStackID==sliderValue
    return;
end
handles.imageStackID=sliderValue;

% plot image stack file
axes(handles.ImageStack_axe);
handles.imageStack_imageHandle=imagesc(handles.imageStack(:,:,handles.imageStackID));
colormap('gray');
daspect([1 1 1]);

% update image ID display
set(handles.ImageStackID_text, 'String', [num2str(handles.imageStackID) ' of ' num2str(handles.imageStackNum)]);

% setup buttondownfcn for image stack
set(handles.imageStack_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ImageStackFocus_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageStackFocus_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function ImageStack_umperframe_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStack_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ImageStack_umperframe_edit as text
%        str2double(get(hObject,'String')) returns contents of ImageStack_umperframe_edit as a double


% --- Executes during object creation, after setting all properties.
function ImageStack_umperframe_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageStack_umperframe_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SelectAxialCalibrationFile_button.
function SelectAxialCalibrationFile_button_Callback(hObject, eventdata, handles)
% hObject    handle to SelectAxialCalibrationFile_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No axial calibration file is selected.','error');
    return;
end

load([pathName fileName]);
handles.f_SLMFocusCalFun=f_SLMFocusCalFun;


function FramePeriod_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FramePeriod_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FramePeriod_edit as text
%        str2double(get(hObject,'String')) returns contents of FramePeriod_edit as a double


% --- Executes during object creation, after setting all properties.
function FramePeriod_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FramePeriod_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ImageClickCallback ( objectHandle , eventData, hObject, handles )
   handles = guidata(hObject);

   if isempty(handles.imageStack_imageHandle) || isempty(handles.activationStack_imageHandle)
       return;
   end
  
   if handles.calibrationTag==0
       return;
   end

% obtain coordinate
   axesHandle  = get(objectHandle,'Parent');
   coordinates = get(axesHandle,'CurrentPoint'); 
   x = coordinates(1,1);
   y = coordinates(1,2);

   if objectHandle==handles.imageStack_imageHandle     % image stack
       if handles.imageStackSelectPointsTag==1         % add points
           handles.imageStackTargetNum=handles.imageStackTargetNum+1;
           handles.imageStackTargetList=[handles.imageStackTargetList; [x y]];
           % delete the target points image handle
           if ~isempty(handles.imageStackTarget_imageHandle)
               delete(handles.imageStackTarget_imageHandle);
               delete(handles.imageStackTarget_labelHandle);
           end
           % replot the target points image
           hold on;
           handles.imageStackTarget_imageHandle=plot(handles.imageStackTargetList(:,1),handles.imageStackTargetList(:,2),'ro');
           handles.imageStackTarget_labelHandle=text(handles.imageStackTargetList(:,1)+3,handles.imageStackTargetList(:,2)-3,num2str([1:handles.imageStackTargetNum]'),'color',[1 0 0],'fontsize',10);
           set(handles.imageStackTarget_imageHandle, 'HitTest','off');
           set(handles.imageStackTarget_labelHandle, 'HitTest','off');                     
       else if handles.imageStackSelectPointsTag==-1   % delete points
           index=find( sqrt((handles.imageStackTargetList(:,1)-x).^2+(handles.imageStackTargetList(:,2)-y).^2) < handles.clickTolerance );  
           if ~isempty(index)
               handles.imageStackTargetNum=handles.imageStackTargetNum-length(index);
               handles.imageStackTargetList(index,:)=[];
           % delete the target points image handle
               delete(handles.imageStackTarget_imageHandle);
               delete(handles.imageStackTarget_labelHandle);
               handles.imageStackTarget_imageHandle=[];
               handles.imageStackTarget_labelHandle=[];
               if handles.imageStackTargetNum>0
            % replot the target points image
                  hold on;
                  handles.imageStackTarget_imageHandle=plot(handles.imageStackTargetList(:,1),handles.imageStackTargetList(:,2),'ro');
                  handles.imageStackTarget_labelHandle=text(handles.imageStackTargetList(:,1)+3,handles.imageStackTargetList(:,2)-3,num2str([1:handles.imageStackTargetNum]'),'color',[1 0 0],'fontsize',10);
                  set(handles.imageStackTarget_imageHandle, 'HitTest','off');
                  set(handles.imageStackTarget_labelHandle, 'HitTest','off');                     
               end
           end
           end
       end
   else if objectHandle==handles.activationStack_imageHandle     % activation stack
       if handles.activationStackSelectPointsTag==1         % add points
           handles.activationStackTargetNum=handles.activationStackTargetNum+1;
           handles.activationStackTargetList=[handles.activationStackTargetList; [x y]];
           % delete the target points image handle
           if ~isempty(handles.activationStackTarget_imageHandle)
               delete(handles.activationStackTarget_imageHandle);
               delete(handles.activationStackTarget_labelHandle);
           end
           % replot the target points image
           hold on;
           handles.activationStackTarget_imageHandle=plot(handles.activationStackTargetList(:,1),handles.activationStackTargetList(:,2),'ro');
           handles.activationStackTarget_labelHandle=text(handles.activationStackTargetList(:,1)+3,handles.activationStackTargetList(:,2)-3,num2str([1:handles.activationStackTargetNum]'),'color',[1 0 0],'fontsize',10);
           set(handles.activationStackTarget_imageHandle, 'HitTest','off');
           set(handles.activationStackTarget_labelHandle, 'HitTest','off');                     
       else if handles.activationStackSelectPointsTag==-1   % delete points
           index=find( sqrt((handles.activationStackTargetList(:,1)-x).^2+(handles.activationStackTargetList(:,2)-y).^2) < handles.clickTolerance );  
           if ~isempty(index)
               handles.activationStackTargetNum=handles.activationStackTargetNum-length(index);
               handles.activationStackTargetList(index,:)=[];
           % delete the target points image handle
               delete(handles.activationStackTarget_imageHandle);
               delete(handles.activationStackTarget_labelHandle);
               handles.activationStackTarget_imageHandle=[];
               handles.activationStackTarget_labelHandle=[];
               if handles.activationStackTargetNum>0
            % replot the target points image
                  hold on;
                  handles.activationStackTarget_imageHandle=plot(handles.activationStackTargetList(:,1),handles.activationStackTargetList(:,2),'ro');
                  handles.activationStackTarget_labelHandle=text(handles.activationStackTargetList(:,1)+3,handles.activationStackTargetList(:,2)-3,num2str([1:handles.activationStackTargetNum]'),'color',[1 0 0],'fontsize',10);
                  set(handles.activationStackTarget_imageHandle, 'HitTest','off');
                  set(handles.activationStackTarget_labelHandle, 'HitTest','off');                     
               end
           end
           end
       end
       end
   end
   guidata(hObject, handles);


% --- Executes on button press in ImageStackAutoZoom_checkbox.
function ImageStackAutoZoom_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackAutoZoom_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ImageStackAutoZoom_checkbox


% --- Executes on button press in ImageStackNext_pushbutton.
function ImageStackNext_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackNext_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.calibrationTag==0
    return;
end

if handles.imageStackTargetID==handles.targetNum
    return;
end

handles.imageStackTargetID=handles.imageStackTargetID+1;
set(handles.ImageStackStatus_text, 'String', [num2str(handles.imageStackTargetID) ' of ' num2str(handles.targetNum)]);

% Zoom into the plot
autoZoom=get(handles.ImageStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.imageStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.imageStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ImageStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ImageStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);



% --- Executes on button press in ImageStackPrevious_pushbutton.
function ImageStackPrevious_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackPrevious_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calibrationTag==0
    return;
end

if handles.imageStackTargetID==1
    return;
end

handles.imageStackTargetID=handles.imageStackTargetID-1;
set(handles.ImageStackStatus_text, 'String', [num2str(handles.imageStackTargetID) ' of ' num2str(handles.targetNum)]);

% Zoom into the plot
autoZoom=get(handles.ImageStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.imageStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.imageStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ImageStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ImageStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);



% --- Executes on button press in ImageStackAutoFind_pushbutton.
function ImageStackAutoFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackAutoFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.calibrationTag==0
    return;
end

interX=handles.imageSize(1)/6;
interY=handles.imageSize(1)/6;
zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.imageStackTargetID-1,6)+1)+handles.imageSize(1)/12;
zoneY=handles.imageSize(1)/6*(floor((handles.imageStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    

if size(handles.imageStackTargetList,1)>=handles.imageStackTargetID
    x1=handles.imageStackTargetList(handles.imageStackTargetID,1)-interX*0.2;
    y1=handles.imageStackTargetList(handles.imageStackTargetID,2)-interY*0.2;
    x2=handles.imageStackTargetList(handles.imageStackTargetID,1)+interX*0.2;
    y2=handles.imageStackTargetList(handles.imageStackTargetID,2)+interY*0.2;
else if size(handles.imageStackTargetList,1)==handles.imageStackTargetID-1
    x1=zoneX-interX*0.5;
    y1=zoneY-interY*0.5;
    x2=zoneX+interX*0.5;
    y2=zoneY+interY*0.5;    
    else
        msgbox('Please select target points in previous zones first','Confirm');
        return;
    end
end
    
if x1<=0 x1=1; end
if y1<=0 y1=1; end
if x2>handles.imageSize(1) x2=handles.imageSize(1); end
if y2>handles.imageSize(1) y2=handles.imageSize(1); end

x1=ceil(x1);
y1=ceil(y1);
x2=floor(x2);
y2=floor(y2);

data=double(handles.imageStack(y1:y2,x1:x2,handles.imageStackID));
[xymax,smax,xymin,smin] = extrema2(data);
[xmin,ymin]=ind2sub([y2-y1+1 x2-x1+1],smin(1));
x=ymin+x1-1;
y=xmin+y1-1;

if size(handles.imageStackTargetList,1)>=handles.imageStackTargetID
    handles.imageStackTargetList(handles.imageStackTargetID,:)=[x y];
else if size(handles.imageStackTargetList,1)==handles.imageStackTargetID-1
        handles.imageStackTargetNum=handles.imageStackTargetNum+1;
        handles.imageStackTargetList=[handles.imageStackTargetList; [x y]];
    end
end

% plot target points for the image
% delete the target points image handle
if ~isempty(handles.imageStackTarget_imageHandle)
    delete(handles.imageStackTarget_imageHandle);
    delete(handles.imageStackTarget_labelHandle);
end
% replot the target points image
hold on;
handles.imageStackTarget_imageHandle=plot(handles.imageStackTargetList(:,1),handles.imageStackTargetList(:,2),'ro');
handles.imageStackTarget_labelHandle=text(handles.imageStackTargetList(:,1)+3,handles.imageStackTargetList(:,2)-3,num2str([1:handles.imageStackTargetNum]'),'color',[1 0 0],'fontsize',10);
set(handles.imageStackTarget_imageHandle, 'HitTest','off');
set(handles.imageStackTarget_labelHandle, 'HitTest','off');                     

% Zoom into the plot
autoZoom=get(handles.ImageStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.imageStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.imageStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ImageStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ImageStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


% --- Executes on button press in ImageStackCenterMassFind_pushbutton.
function ImageStackCenterMassFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackCenterMassFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.calibrationTag==0
    return;
end

if size(handles.imageStackTargetList,1)<handles.imageStackTargetID
    return;
end

x0=round(handles.imageStackTargetList(handles.imageStackTargetID,1));
y0=round(handles.imageStackTargetList(handles.imageStackTargetID,2));

centerOfMassRange=str2num(get(handles.ImageStackCenterOfMassRange_edit,'String'));
centerOfMassHalfRange=floor(centerOfMassRange/2);
centerOfMassThreshold=str2num(get(handles.ImageStackCenterOfMassThreshold_edit,'String'));

[tempX tempY]=meshgrid(x0-centerOfMassHalfRange:x0+centerOfMassHalfRange, ...
    y0-centerOfMassHalfRange:y0+centerOfMassHalfRange);

data=handles.imageStack(y0-centerOfMassHalfRange:y0+centerOfMassHalfRange, ...
    x0-centerOfMassHalfRange:x0+centerOfMassHalfRange,handles.imageStackID);
data=double(data);
index=find(data<(mean(data(:))-centerOfMassThreshold*std(data(:))));
if isempty(index)
    return;
end

xc=sum(tempX(index).*data(index))/sum(data(index));
yc=sum(tempY(index).*data(index))/sum(data(index));

handles.imageStackTargetList(handles.imageStackTargetID,:)=[xc yc];

% plot target points for the image
% delete the target points image handle
if ~isempty(handles.imageStackTarget_imageHandle)
    delete(handles.imageStackTarget_imageHandle);
    delete(handles.imageStackTarget_labelHandle);
end
% replot the target points image
hold on;
handles.imageStackTarget_imageHandle=plot(handles.imageStackTargetList(:,1),handles.imageStackTargetList(:,2),'ro');
handles.imageStackTarget_labelHandle=text(handles.imageStackTargetList(:,1)+3,handles.imageStackTargetList(:,2)-3,num2str([1:handles.imageStackTargetNum]'),'color',[1 0 0],'fontsize',10);
set(handles.imageStackTarget_imageHandle, 'HitTest','off');
set(handles.imageStackTarget_labelHandle, 'HitTest','off');                     

% Zoom into the plot
autoZoom=get(handles.ImageStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.imageStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.imageStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ImageStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ImageStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


function ImageStackCenterOfMassRange_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackCenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ImageStackCenterOfMassRange_edit as text
%        str2double(get(hObject,'String')) returns contents of ImageStackCenterOfMassRange_edit as a double


% --- Executes during object creation, after setting all properties.
function ImageStackCenterOfMassRange_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageStackCenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ImageStackCenterOfMassThreshold_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ImageStackCenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ImageStackCenterOfMassThreshold_edit as text
%        str2double(get(hObject,'String')) returns contents of ImageStackCenterOfMassThreshold_edit as a double


% --- Executes during object creation, after setting all properties.
function ImageStackCenterOfMassThreshold_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ImageStackCenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ActivationStackAutoZoom_checkbox.
function ActivationStackAutoZoom_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackAutoZoom_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ActivationStackAutoZoom_checkbox


% --- Executes on button press in ActivationStackNext_pushbutton.
function ActivationStackNext_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackNext_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calibrationTag==0
    return;
end

if handles.activationStackTargetID==handles.targetNum
    return;
end

handles.activationStackTargetID=handles.activationStackTargetID+1;
set(handles.ActivationStackStatus_text, 'String', [num2str(handles.activationStackTargetID) ' of ' num2str(handles.targetNum)]);

% Zoom into the plot
autoZoom=get(handles.ActivationStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.activationStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.activationStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ActivationStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ActivationStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);



% --- Executes on button press in ActivationStackPrevious_pushbutton.
function ActivationStackPrevious_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackPrevious_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calibrationTag==0
    return;
end

if handles.activationStackTargetID==1
    return;
end

handles.activationStackTargetID=handles.activationStackTargetID-1;
set(handles.ActivationStackStatus_text, 'String', [num2str(handles.activationStackTargetID) ' of ' num2str(handles.targetNum)]);

% Zoom into the plot
autoZoom=get(handles.ActivationStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.activationStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.activationStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ActivationStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ActivationStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


% --- Executes on button press in ActivationStackAutoFind_pushbutton.
function ActivationStackAutoFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackAutoFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calibrationTag==0
    return;
end

interX=handles.imageSize(1)/6;
interY=handles.imageSize(1)/6;
zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.activationStackTargetID-1,6)+1)+handles.imageSize(1)/12;
zoneY=handles.imageSize(1)/6*(floor((handles.activationStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    

if size(handles.activationStackTargetList,1)>=handles.activationStackTargetID
    x1=handles.activationStackTargetList(handles.activationStackTargetID,1)-interX*0.2;
    y1=handles.activationStackTargetList(handles.activationStackTargetID,2)-interY*0.2;
    x2=handles.activationStackTargetList(handles.activationStackTargetID,1)+interX*0.2;
    y2=handles.activationStackTargetList(handles.activationStackTargetID,2)+interY*0.2;
else if size(handles.activationStackTargetList,1)==handles.activationStackTargetID-1
    x1=zoneX-interX*0.5;
    y1=zoneY-interY*0.5;
    x2=zoneX+interX*0.5;
    y2=zoneY+interY*0.5;    
    else
        msgbox('Please select target points in previous zones first','Confirm');
        return;
    end
end
    
if x1<=0 x1=1; end
if y1<=0 y1=1; end
if x2>handles.imageSize(1) x2=handles.imageSize(1); end
if y2>handles.imageSize(1) y2=handles.imageSize(1); end

x1=ceil(x1);
y1=ceil(y1);
x2=floor(x2);
y2=floor(y2);

data=imrotate(handles.activationStack(:,:,handles.activationStackID),handles.imageRotation);
data=double(data(y1:y2,x1:x2));
[xymax,smax,xymin,smin] = extrema2(data);
[xmin,ymin]=ind2sub([y2-y1+1 x2-x1+1],smin(1));
x=ymin+x1-1;
y=xmin+y1-1;

if size(handles.activationStackTargetList,1)>=handles.activationStackTargetID
    handles.activationStackTargetList(handles.activationStackTargetID,:)=[x y];
else if size(handles.activationStackTargetList,1)==handles.activationStackTargetID-1
        handles.activationStackTargetNum=handles.activationStackTargetNum+1;
        handles.activationStackTargetList=[handles.activationStackTargetList; [x y]];
    end
end

% plot target points for the image
% delete the target points image handle
if ~isempty(handles.activationStackTarget_imageHandle)
    delete(handles.activationStackTarget_imageHandle);
    delete(handles.activationStackTarget_labelHandle);
end
% replot the target points image
hold on;
handles.activationStackTarget_imageHandle=plot(handles.activationStackTargetList(:,1),handles.activationStackTargetList(:,2),'ro');
handles.activationStackTarget_labelHandle=text(handles.activationStackTargetList(:,1)+3,handles.activationStackTargetList(:,2)-3,num2str([1:handles.activationStackTargetNum]'),'color',[1 0 0],'fontsize',10);
set(handles.activationStackTarget_imageHandle, 'HitTest','off');
set(handles.activationStackTarget_labelHandle, 'HitTest','off');                     

% Zoom into the plot
autoZoom=get(handles.ActivationStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.activationStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.activationStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ActivationStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ActivationStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


% --- Executes on button press in ActivationStackCenterMassFind_pushbutton.
function ActivationStackCenterMassFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackCenterMassFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.calibrationTag==0
    return;
end

if size(handles.activationStackTargetList,1)<handles.activationStackTargetID
    return;
end

x0=round(handles.activationStackTargetList(handles.activationStackTargetID,1));
y0=round(handles.activationStackTargetList(handles.activationStackTargetID,2));

centerOfMassRange=str2num(get(handles.ActivationStackCenterOfMassRange_edit,'String'));
centerOfMassHalfRange=floor(centerOfMassRange/2);
centerOfMassThreshold=str2num(get(handles.ActivationStackCenterOfMassThreshold_edit,'String'));

[tempX tempY]=meshgrid(x0-centerOfMassHalfRange:x0+centerOfMassHalfRange, ...
    y0-centerOfMassHalfRange:y0+centerOfMassHalfRange);

data=imrotate(handles.activationStack(:,:,handles.activationStackID),handles.imageRotation);
data=double(data(y0-centerOfMassHalfRange:y0+centerOfMassHalfRange, x0-centerOfMassHalfRange:x0+centerOfMassHalfRange));
index=find(data<(mean(data(:))-centerOfMassThreshold*std(data(:))));
if isempty(index)
    return;
end

xc=sum(tempX(index).*data(index))/sum(data(index));
yc=sum(tempY(index).*data(index))/sum(data(index));

handles.activationStackTargetList(handles.activationStackTargetID,:)=[xc yc];

% plot target points for the image
% delete the target points image handle
if ~isempty(handles.activationStackTarget_imageHandle)
    delete(handles.activationStackTarget_imageHandle);
    delete(handles.activationStackTarget_labelHandle);
end
% replot the target points image
hold on;
handles.activationStackTarget_imageHandle=plot(handles.activationStackTargetList(:,1),handles.activationStackTargetList(:,2),'ro');
handles.activationStackTarget_labelHandle=text(handles.activationStackTargetList(:,1)+3,handles.activationStackTargetList(:,2)-3,num2str([1:handles.activationStackTargetNum]'),'color',[1 0 0],'fontsize',10);
set(handles.activationStackTarget_imageHandle, 'HitTest','off');
set(handles.activationStackTarget_labelHandle, 'HitTest','off');                     

% Zoom into the plot
autoZoom=get(handles.ActivationStackAutoZoom_checkbox, 'Value');
if autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.activationStackTargetID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.activationStackTargetID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ActivationStack_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ActivationStack_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(2)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


function ActivationStackCenterOfMassRange_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackCenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationStackCenterOfMassRange_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationStackCenterOfMassRange_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationStackCenterOfMassRange_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStackCenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ActivationStackCenterOfMassThreshold_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationStackCenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationStackCenterOfMassThreshold_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationStackCenterOfMassThreshold_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationStackCenterOfMassThreshold_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationStackCenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ActivationLaserImageRotation_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationLaserImageRotation_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationLaserImageRotation_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationLaserImageRotation_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationLaserImageRotation_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationLaserImageRotation_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
