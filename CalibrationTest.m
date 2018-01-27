% To test the lateral calibration.
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationTest(varargin)
% CALIBRATIONTEST MATLAB code for CalibrationTest.fig
%      CALIBRATIONTEST, by itself, creates a new CALIBRATIONTEST or raises the existing
%      singleton*.
%
%      H = CALIBRATIONTEST returns the handle to a new CALIBRATIONTEST or the handle to
%      the existing singleton*.
%
%      CALIBRATIONTEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALIBRATIONTEST.M with the given input arguments.
%
%      CALIBRATIONTEST('Property','Value',...) creates a new CALIBRATIONTEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CalibrationTest_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CalibrationTest_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationTest

% Last Modified by GUIDE v2.5 02-Nov-2015 23:41:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationTest_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationTest_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before CalibrationTest is made visible.
function CalibrationTest_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CalibrationTest (see VARARGIN)

% Choose default command line output for CalibrationTest
handles.output = hObject;

% self defined parameters
handles.parentHandles=varargin{1};  % handles from MainControl.m
handles.SLM_handles=handles.parentHandles.SLM_handles;    % SLM handle
handles.XStepSize = 10;             % [pixel] X step size 
handles.YStepSize = 10;             % [pixel] Y step size
handles.ZStepSize = 10;             % [um] Z step size
handles.XOffset = 0;                % [pixel] X offset
handles.YOffset = 0;                % [pixel] Y offset
handles.ZOffset = 0;                % [um] Z offset
handles.XYZChoice = [1 1 0];        % XYZ choice, default is [1 1 0]

handles.activationTime = 0.5;       % [s] activation time for each spot

handles.targetList = [];            % target list
handles.resultList = [];            % result list

handles.image = [];                 % targeting result image
handles.imageSize = [];             % targeting result image size
handles.imageHandle = [];           % targeting result image handle

handles.targetID = [];               % current target ID
handles.resultList_imageHandle = []; % handle to plot the clicked targets

handles.SLMPreset=handles.parentHandles.SLMPreset;

handles.startMeasurement = 0;        % indicator for whether measurement is started

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationTest wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationTest_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in XDirection_radiobutton.
function XDirection_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to XDirection_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of XDirection_radiobutton


% --- Executes on button press in YDirection_radiobutton.
function YDirection_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to YDirection_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of YDirection_radiobutton


% --- Executes on button press in ZDirection_radiobutton.
function ZDirection_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to ZDirection_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ZDirection_radiobutton



function XStepSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to XStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of XStepSize_edit as text
%        str2double(get(hObject,'String')) returns contents of XStepSize_edit as a double


% --- Executes during object creation, after setting all properties.
function XStepSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YStepSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to YStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YStepSize_edit as text
%        str2double(get(hObject,'String')) returns contents of YStepSize_edit as a double


% --- Executes during object creation, after setting all properties.
function YStepSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ZStepSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ZStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ZStepSize_edit as text
%        str2double(get(hObject,'String')) returns contents of ZStepSize_edit as a double


% --- Executes during object creation, after setting all properties.
function ZStepSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ZStepSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in StartCalibrationTest_pushbutton.
function StartCalibrationTest_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to StartCalibrationTest_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.XYZChoice(1)=get(handles.XDirection_radiobutton,'Value');
handles.XYZChoice(2)=get(handles.YDirection_radiobutton,'Value');
handles.XYZChoice(3)=get(handles.ZDirection_radiobutton,'Value');

handles.XStepSize = str2num(get(handles.XStepSize_edit,'String'));
handles.YStepSize = str2num(get(handles.YStepSize_edit,'String'));
handles.ZStepSize = str2num(get(handles.ZStepSize_edit,'String'));
handles.XOffset = str2num(get(handles.XOffset_edit,'String'));                
handles.YOffset = str2num(get(handles.YOffset_edit,'String'));                
handles.ZOffset = str2num(get(handles.ZOffset_edit,'String'));                

handles.ActivationTime = str2num(get(handles.ActivationTime_edit,'String'));
handles.ActivationTime = handles.ActivationTime/1000;

if handles.XYZChoice==[1 1 0]
    experimentImageSize=size(handles.parentHandles.imageStack,1);
    xNum=round(experimentImageSize/handles.XStepSize);
    yNum=round(experimentImageSize/handles.YStepSize);    
    [X Y]=meshgrid(linspace(experimentImageSize*0.05, experimentImageSize*0.95, xNum), linspace(experimentImageSize*0.05, experimentImageSize*0.95, yNum)); 
    X=round(X(:))+handles.XOffset;
    Y=round(Y(:))+handles.YOffset;
    Z=zeros(length(X),1)+handles.ZOffset*1e-6;
    handles.targetList=[X Y Z];    
else
    msgbox('XZ step or YZ step is not supported yet.','Error');
end


%% start SLM activation: This program is similar to "function SLMActivation_button_Callback(hObject, eventdata, handles)" in MainControl.m

% ROI coordinate conversion
xyImage=handles.targetList(:,1:2);
zPosition=handles.targetList(:,3);

% scale the xyImage coordinate based on the image pixel size (e.g. whether it is 512x512 or 256x256 image size, etc)
experimentImageSize=size(handles.parentHandles.imageStack,1);
scaleFactor=experimentImageSize/handles.parentHandles.calImageSize(1);
xyImage=xyImage/scaleFactor;

% Galvo center point to correct the target point on xy activation
% activation laser center point
activationCenter=handles.parentHandles.calImageSize/2;

% image laser center point, find the calibration file at 0 um plane
[dump, zPositionID]=min(abs(handles.parentHandles.tImage2Activation_focusPlane-0));
imageCenter=tformfwd(handles.parentHandles.tImage2Activation{zPositionID}(end), activationCenter);

% correction factor
% handles.parentHandles.imageCorrectionFactorX=1.00;
% handles.parentHandles.imageCorrectionFactorY=1.02;
xyImage(:,1)=(xyImage(:,1)-imageCenter(1)*scaleFactor)*handles.parentHandles.imageCorrectionFactorX+imageCenter(1)*scaleFactor;
xyImage(:,2)=(xyImage(:,2)-imageCenter(2)*scaleFactor)*handles.parentHandles.imageCorrectionFactorY+imageCenter(2)*scaleFactor;

%% Construct 25 point zone
% 25 point zone
[zoneX zoneY]=meshgrid(handles.parentHandles.calImageSize(1)-handles.parentHandles.calImageSize(1)/6:-handles.parentHandles.calImageSize(1)/6:handles.parentHandles.calImageSize(1)/6, ...
                       handles.parentHandles.calImageSize(1)/6:handles.parentHandles.calImageSize(1)/6:handles.parentHandles.calImageSize(1)-handles.parentHandles.calImageSize(1)/6);
zoneX=zoneX'; zoneX=zoneX(:);
zoneY=zoneY'; zoneY=zoneY(:);

%% Check whether imaging stack or activation stack is loaded
if handles.parentHandles.stackTag==1      % activation stack, no need to do xyImage to xyActivation transfer
    xyActivation=xyImage;
else                % image stack, need to do a transform from xy image to xy activation    
% the following consider different focus for tImage2Activation, use 'linearinterp' for fitting
    xyActivation=zeros(size(xyImage));
    for idx=1:size(xyImage,1)
        [dump, zPositionID]=min(abs(handles.parentHandles.tImage2Activation_focusPlane-zPosition(idx)));
        [dump, zoneID]=min((zoneX-xyImage(idx,1)).^2+(zoneY-xyImage(idx,2)).^2);
        
        if zPositionID==1
            tempzID=[1 2];
        else if zPositionID==length(handles.parentHandles.tImage2Activation_focusPlane)
                tempzID=[length(handles.parentHandles.tImage2Activation_focusPlane)-1 length(handles.parentHandles.tImage2Activation_focusPlane)];
            else
                tempzID=[zPositionID-1 zPositionID zPositionID+1];
            end
        end
            
        for idx1=1:length(tempzID)
            xyTemp(idx1,:)=tforminv(handles.parentHandles.tImage2Activation{tempzID(idx1)}(zoneID),xyImage(idx,:));
        end
        
        [calFun1,dump1,dump2] = fit( handles.parentHandles.tImage2Activation_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
        [calFun2,dump1,dump2] = fit( handles.parentHandles.tImage2Activation_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

        xyActivation(idx,1)=calFun1(zPosition(idx));
        xyActivation(idx,2)=calFun2(zPosition(idx));        
    end
end

%% correct xy activation because we use the laser for activation instead of imaging
xyActivationCorrected=xyActivation;
xyActivationCorrected(:,1)=handles.parentHandles.calImageSize(1)+1-xyActivation(:,1);
xyActivationCorrected(:,2)=handles.parentHandles.calImageSize(1)+1-xyActivation(:,2);

% transform from xy activation to xyp plane
xyp=zeros(size(xyActivationCorrected));
                   
% the following consider different focus for tActivationLaserSLM, use 'linearinterp' for fitting 
for idx=1:size(xyp,1)
    [dump, zPositionID]=min(abs(handles.parentHandles.tActivationLaserSLM_focusPlane-zPosition(idx)));
    [dump, zoneID]=min((zoneX-xyActivationCorrected(idx,1)).^2+(zoneY-xyActivationCorrected(idx,2)).^2);
    xyp(idx,:)=tforminv(handles.parentHandles.tActivationLaserSLM{zPositionID}(zoneID),xyActivationCorrected(idx,:));
    
    if zPositionID==1
        tempzID=[1 2];
    else if zPositionID==length(handles.parentHandles.tActivationLaserSLM_focusPlane)
            tempzID=[length(handles.parentHandles.tActivationLaserSLM_focusPlane)-1 length(handles.parentHandles.tActivationLaserSLM_focusPlane)];
        else
            tempzID=[zPositionID-1 zPositionID zPositionID+1];
        end
    end
            
    clear xyTemp;
    for idx1=1:length(tempzID)
        xyTemp(idx1,:)=tforminv(handles.parentHandles.tActivationLaserSLM{tempzID(idx1)}(zoneID),xyActivationCorrected(idx,:));
    end
        
    [calFun1,dump1,dump2] = fit( handles.parentHandles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
    [calFun2,dump1,dump2] = fit( handles.parentHandles.tActivationLaserSLM_focusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

    xyp(idx,1)=calFun1(zPosition(idx));
    xyp(idx,2)=calFun2(zPosition(idx));        
end

% add the z column
xyzp=zeros(size(xyp,1),3);
xyzp(:,1:2)=xyp;
xyzp(:,3)=zPosition;
objectiveNA=handles.parentHandles.f_SLMFocusCalFun(xyzp(:,3));
weight=ones(1,size(xyzp,1));

%% start NI instrument and actuate SLM
session = daq.createSession ('ni');
session.addDigitalChannel('dev1','Port0/Line0','OutputOnly');
session.outputSingleScan(0);
waitfor(msgbox(['Please set up Prairie for ' num2str(size(xyzp,1)) ' of activation, and connect D0 port in NI box to Prairie input trigger. Set up Prairie activation "Trigger" to be "TrigIn", and "WaitforTrigger" as "EveryRepetition". Start Prairie Acquisition. Press OK when ready.']));
set(handles.StartCalibrationTest_pushbutton,'enable','off');

for idx=1:size(xyzp,1)
     f_SLMActivation_Calibration( handles.SLM_handles, xyzp(idx,:), weight(idx), objectiveNA(idx) );
     pause(0.1);
      session.outputSingleScan(1);
     pause(0.1);
      session.outputSingleScan(0);
     set(handles.Status_text, 'String', [num2str(idx) ' of ' num2str(size(xyzp,1))]);
     pause(handles.activationTime);
end
set(handles.StartCalibrationTest_pushbutton,'enable','on');

f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], 1, 0.4 );
guidata(hObject,handles);


function XOffset_edit_Callback(hObject, eventdata, handles)
% hObject    handle to XOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of XOffset_edit as text
%        str2double(get(hObject,'String')) returns contents of XOffset_edit as a double


% --- Executes during object creation, after setting all properties.
function XOffset_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YOffset_edit_Callback(hObject, eventdata, handles)
% hObject    handle to YOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YOffset_edit as text
%        str2double(get(hObject,'String')) returns contents of YOffset_edit as a double


% --- Executes during object creation, after setting all properties.
function YOffset_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ZOffset_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ZOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ZOffset_edit as text
%        str2double(get(hObject,'String')) returns contents of ZOffset_edit as a double


% --- Executes during object creation, after setting all properties.
function ZOffset_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ZOffset_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TargetingResultImage_edit_Callback(hObject, eventdata, handles)
% hObject    handle to TargetingResultImage_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TargetingResultImage_edit as text
%        str2double(get(hObject,'String')) returns contents of TargetingResultImage_edit as a double


% --- Executes during object creation, after setting all properties.
function TargetingResultImage_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TargetingResultImage_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Browse_pushbutton.
function Browse_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Browse_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% read targeting result image file
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.TargetingResultImage_edit, 'String', [pathName fileName]);
handles.image=f_readTiffFile([pathName fileName]);
handles.imageSize=[size(handles.image,1) size(handles.image,2)];

% plot image file
axes(handles.Image_axes);
handles.imageHandle=imagesc(handles.image);
colormap('gray');
daspect([1 1 1]);

% setup buttondownfcn for shifted image
set(handles.imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% initialize handles.resultList
handles.resultList=ones(size(handles.targetList,1),2);
handles.resultList=handles.resultList*-1;

guidata(hObject,handles);


function ActivationTime_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ActivationTime_edit as text
%        str2double(get(hObject,'String')) returns contents of ActivationTime_edit as a double


% --- Executes during object creation, after setting all properties.
function ActivationTime_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ActivationTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in StartMeasurement_pushbutton.
function StartMeasurement_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to StartMeasurement_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.startMeasurement=1;
handles.targetID=1;
set(handles.Status_text, 'String', [num2str(handles.targetID) ' of ' num2str(size(handles.targetList,1))]);

% setup buttondownfcn for image
set(handles.imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% zoom into image
interX=max(diff(handles.targetList(:,1)));
interY=max(diff(handles.targetList(:,2)));

x1=handles.targetList(handles.targetID,1)-interX*0.8;
y1=handles.targetList(handles.targetID,2)-interY*0.8;
x2=handles.targetList(handles.targetID,1)+interX*0.8;
y2=handles.targetList(handles.targetID,2)+interY*0.8;

if get(handles.AutoZoom_radiobutton,'Value') == 1
    axis([x1 x2 y1 y2]);
else
    axis([1 handles.imageSize(2) 1 handles.imageSize(1)]);
end

axes(handles.Image_axes);
hold on;
for idx=1:3
    targetList_imageHandle=plot(handles.targetList(handles.targetID,1)-handles.XOffset,handles.targetList(handles.targetID,2)-handles.YOffset,'b*');
    pause(0.2);
    delete(targetList_imageHandle);
    pause(0.2);
end

guidata(hObject, handles);


% --- Executes on button press in Next_pushbutton.
function Next_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Next_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.targetID==size(handles.targetList,1)
    return;
end

handles.targetID=handles.targetID+1;
set(handles.Status_text, 'String', [num2str(handles.targetID) ' of ' num2str(size(handles.targetList,1))]);

% setup buttondownfcn for image
set(handles.imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% zoom into image
interX=max(diff(handles.targetList(:,1)));
interY=max(diff(handles.targetList(:,2)));

x1=handles.targetList(handles.targetID,1)-interX*0.8;
y1=handles.targetList(handles.targetID,2)-interY*0.8;
x2=handles.targetList(handles.targetID,1)+interX*0.8;
y2=handles.targetList(handles.targetID,2)+interY*0.8;

if get(handles.AutoZoom_radiobutton,'Value') == 1
    axis([x1 x2 y1 y2]);
else
    axis([1 handles.imageSize(2) 1 handles.imageSize(1)]);
end

axes(handles.Image_axes);
hold on;
if handles.resultList(handles.targetID,1)~=-1
    if isempty(handles.resultList_imageHandle)
        handles.resultList_imageHandle=plot(handles.resultList(handles.targetID,1),handles.resultList(handles.targetID,2),'ro');
    else
        delete(handles.resultList_imageHandle);
        handles.resultList_imageHandle=plot(handles.resultList(handles.targetID,1),handles.resultList(handles.targetID,2),'ro');
    end
end
    
for idx=1:3
    targetList_imageHandle=plot(handles.targetList(handles.targetID,1)-handles.XOffset,handles.targetList(handles.targetID,2)-handles.YOffset,'b*');
    pause(0.2);
    delete(targetList_imageHandle);
    pause(0.2);
end

guidata(hObject, handles);


% --- Executes on button press in Previous_pushbutton.
function Previous_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Previous_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.targetID)
    return;
end

if handles.targetID==1
    return;
end

handles.targetID=handles.targetID-1;
set(handles.Status_text, 'String', [num2str(handles.targetID) ' of ' num2str(size(handles.targetList,1))]);

% setup buttondownfcn for image
set(handles.imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% zoom into image
interX=max(diff(handles.targetList(:,1)));
interY=max(diff(handles.targetList(:,2)));

x1=handles.targetList(handles.targetID,1)-interX*0.8;
y1=handles.targetList(handles.targetID,2)-interY*0.8;
x2=handles.targetList(handles.targetID,1)+interX*0.8;
y2=handles.targetList(handles.targetID,2)+interY*0.8;

if get(handles.AutoZoom_radiobutton,'Value') == 1
    axis([x1 x2 y1 y2]);
else
    axis([1 handles.imageSize(2) 1 handles.imageSize(1)]);
end

axes(handles.Image_axes);
hold on;
if handles.resultList(handles.targetID,1)~=-1
    if isempty(handles.resultList_imageHandle)
        handles.resultList_imageHandle=plot(handles.resultList(handles.targetID,1),handles.resultList(handles.targetID,2),'ro');
    else
        delete(handles.resultList_imageHandle);
        handles.resultList_imageHandle=plot(handles.resultList(handles.targetID,1),handles.resultList(handles.targetID,2),'ro');
    end
end
    
for idx=1:3
    targetList_imageHandle=plot(handles.targetList(handles.targetID,1)-handles.XOffset,handles.targetList(handles.targetID,2)-handles.YOffset,'b*');
    pause(0.2);
    delete(targetList_imageHandle);
    pause(0.2);
end

guidata(hObject, handles);


% --- Executes on button press in SaveMeasurement_pushbutton.
function SaveMeasurement_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to SaveMeasurement_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[file,path] = uiputfile('*.mat','Save Calibration Test File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration Test file cannot be saved', 'Error');
    return;
end

targetList=handles.targetList;
resultList=handles.resultList;
XYZChoice=handles.XYZChoice;
imageSize=handles.imageSize;

XStepSize=handles.XStepSize;
YStepSize=handles.YStepSize;
ZStepSize=handles.ZStepSize;
XOffset=handles.XOffset;                
YOffset=handles.YOffset;                
ZOffset=handles.ZOffset;    

save([path file],'targetList','resultList','XYZChoice','imageSize','XStepSize','YStepSize','ZStepSize','XOffset','YOffset','ZOffset');
msgbox('Calibration file Saved.','Confirm');


% --- Executes on button press in AutoFind_pushbutton.
function AutoFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to AutoFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% image boundary
interX=max(diff(handles.targetList(:,1)));
interY=max(diff(handles.targetList(:,2)));

if handles.resultList(handles.targetID,1)==-1
    x1=handles.targetList(handles.targetID,1)-interX*0.5;
    y1=handles.targetList(handles.targetID,2)-interY*0.5;
    x2=handles.targetList(handles.targetID,1)+interX*0.5;
    y2=handles.targetList(handles.targetID,2)+interY*0.5;
else
    x1=handles.resultList(handles.targetID,1)-interX*0.2;
    y1=handles.resultList(handles.targetID,2)-interY*0.2;
    x2=handles.resultList(handles.targetID,1)+interX*0.2;
    y2=handles.resultList(handles.targetID,2)+interY*0.2;    
end

if x1<0 x1=1; end
if y1<0 y1=1; end
if x2>handles.imageSize(1) x2=handles.imageSize(1); end
if y2>handles.imageSize(2) y2=handles.imageSize(2); end

x1=round(x1);
y1=round(y1);
x2=round(x2);
y2=round(y2);

data=double(handles.image(y1:y2,x1:x2));
[xymax,smax,xymin,smin] = extrema2(data);
[xmin,ymin]=ind2sub([y2-y1+1 x2-x1+1],smin(1));
x=ymin+x1-1;
y=xmin+y1-1;

axes(handles.Image_axes);
if isempty(handles.resultList_imageHandle)
    hold on;
    handles.resultList_imageHandle=plot(x,y,'ro');
else
    delete(handles.resultList_imageHandle);
    hold on;
    handles.resultList_imageHandle=plot(x,y,'ro');
end    
handles.resultList(handles.targetID,:)=[x y];

guidata(hObject, handles);


% --- Executes on button press in Exit_pushbutton.
function Exit_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

hf=findobj('Name','CalibrationTest');
delete(hf);


function ImageClickCallback ( objectHandle, eventData, hObject, handles )
handles = guidata(hObject);

if isempty(handles.imageHandle)
   return;
end
   
if handles.startMeasurement == 0
    return;
end

axesHandle  = get(objectHandle,'Parent');
coordinates = get(axesHandle,'CurrentPoint'); 
x = coordinates(1,1);
y = coordinates(1,2);
if isempty(handles.resultList_imageHandle)
    hold on;
    handles.resultList_imageHandle=plot(x,y,'ro');
else
    delete(handles.resultList_imageHandle);
    hold on;
    handles.resultList_imageHandle=plot(x,y,'ro');
end    
handles.resultList(handles.targetID,:)=[x y];

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function Image_axes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Image_axes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate Image_axes


% --- Executes on button press in AutoZoom_radiobutton.
function AutoZoom_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to AutoZoom_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of AutoZoom_radiobutton


% --- Executes on button press in CenterMassFind_pushbutton.
function CenterMassFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to CenterMassFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.resultList(handles.targetID,1)==-1
    return;
end

x0=round(handles.resultList(handles.targetID,1));
y0=round(handles.resultList(handles.targetID,2));

centerOfMassRange=str2num(get(handles.CenterOfMassRange_edit,'String'));
centerOfMassHalfRange=floor(centerOfMassRange/2);
centerOfMassThreshold=str2num(get(handles.CenterOfMassThreshold_edit,'String'));

[tempX tempY]=meshgrid(x0-centerOfMassHalfRange:x0+centerOfMassHalfRange, ...
    y0-centerOfMassHalfRange:y0+centerOfMassHalfRange);
data=handles.image(y0-centerOfMassHalfRange:y0+centerOfMassHalfRange, ...
    x0-centerOfMassHalfRange:x0+centerOfMassHalfRange);
data=double(data);
index=find(data<(mean(data(:))-centerOfMassThreshold*std(data(:))));
if isempty(index)
    return;
end

xc=sum(tempX(index).*data(index))/sum(data(index));
yc=sum(tempY(index).*data(index))/sum(data(index));

handles.resultList(handles.targetID,1)=xc;
handles.resultList(handles.targetID,2)=yc;

axes(handles.Image_axes);
if isempty(handles.resultList_imageHandle)
    hold on;
    handles.resultList_imageHandle=plot(xc,yc,'ro');
else
    delete(handles.resultList_imageHandle);
    hold on;
    handles.resultList_imageHandle=plot(xc,yc,'ro');
end    

guidata(hObject, handles);



function CenterOfMassRange_edit_Callback(hObject, eventdata, handles)
% hObject    handle to CenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CenterOfMassRange_edit as text
%        str2double(get(hObject,'String')) returns contents of CenterOfMassRange_edit as a double


% --- Executes during object creation, after setting all properties.
function CenterOfMassRange_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CenterOfMassRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CenterOfMassThreshold_edit_Callback(hObject, eventdata, handles)
% hObject    handle to CenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CenterOfMassThreshold_edit as text
%        str2double(get(hObject,'String')) returns contents of CenterOfMassThreshold_edit as a double


% --- Executes during object creation, after setting all properties.
function CenterOfMassThreshold_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CenterOfMassThreshold_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
