% Lateral calibration: to calibrate the lateral deflection by SLM
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationControl_LateralCalibration(varargin)
%CALIBRATIONCONTROL_LATERALCALIBRATION M-file for CalibrationControl_LateralCalibration.fig
%      CALIBRATIONCONTROL_LATERALCALIBRATION, by itself, creates a new CALIBRATIONCONTROL_LATERALCALIBRATION or raises the existing
%      singleton*.
%
%      H = CALIBRATIONCONTROL_LATERALCALIBRATION returns the handle to a new CALIBRATIONCONTROL_LATERALCALIBRATION or the handle to
%      the existing singleton*.
%
%      CALIBRATIONCONTROL_LATERALCALIBRATION('Property','Value',...) creates a new CALIBRATIONCONTROL_LATERALCALIBRATION using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to CalibrationControl_LateralCalibration_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CALIBRATIONCONTROL_LATERALCALIBRATION('CALLBACK') and CALIBRATIONCONTROL_LATERALCALIBRATION('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CALIBRATIONCONTROL_LATERALCALIBRATION.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationControl_LateralCalibration

% Last Modified by GUIDE v2.5 03-Nov-2015 22:57:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationControl_LateralCalibration_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationControl_LateralCalibration_OutputFcn, ...
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


% --- Executes just before CalibrationControl_LateralCalibration is made visible.
function CalibrationControl_LateralCalibration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for CalibrationControl_LateralCalibration
handles.output = hObject;

% Self defined parameters
handles.SLM_handles=varargin{1};    % SLM handle
CFG;
handles.SLMPreset=SLMPreset;
handles.objectiveNA=0.6;
handles.acqImageID=0;
handles.xyzp=[];

handles.refImage=[];
handles.shiftedImage=[];
handles.calImageID=0;
handles.imageNum=0;
handles.imageSize=[];
handles.referenceImage_imageHandle=[];
handles.shiftedImage_imageHandle=[];
handles.refImageTarget_imageHandle=[];
handles.shiftedImageTarget_imageHandle=[];
handles.refImageTargetList=[];
handles.shiftedImageTargetList=[];

handles.tActivationLaserSLM=[];

handles.currentFocus=0;
handles.f_SLMFocusCalFun=[];

handles.autoZoom=0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationControl_LateralCalibration wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationControl_LateralCalibration_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function ReferenceImageFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ReferenceImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ReferenceImageFile_edit as text
%        str2double(get(hObject,'String')) returns contents of ReferenceImageFile_edit as a double


% --- Executes during object creation, after setting all properties.
function ReferenceImageFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ReferenceImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseReferenceImage_button.
function BrowseReferenceImage_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseReferenceImage_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% read reference image file
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ReferenceImageFile_edit, 'String', [pathName fileName]);
handles.refImage=f_readTiffFile([pathName fileName]);
handles.imageSize=[size(handles.refImage,1) size(handles.refImage,2)];

% plot reference image file
axes(handles.ReferenceImage_axe);
handles.referenceImage_imageHandle=imagesc(handles.refImage);
colormap('gray');
daspect([1 1 1]);

% setup buttondownfcn for shifted image
set(handles.referenceImage_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

guidata(hObject, handles);
      

function ShiftedImageFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ShiftedImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ShiftedImageFile_edit as text
%        str2double(get(hObject,'String')) returns contents of ShiftedImageFile_edit as a double


% --- Executes during object creation, after setting all properties.
function ShiftedImageFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ShiftedImageFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseShiftedImage_button.
function BrowseShiftedImage_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseShiftedImage_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% read shifted image file
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ShiftedImageFile_edit, 'String', [pathName fileName]);
handles.shiftedImage=f_readTiffFile([pathName fileName]);

handles.calImageID=1;
handles.imageNum=size(handles.shiftedImage,3);

% plot shifted image file
axes(handles.ShiftedImage_axe);
handles.shiftedImage_imageHandle=imagesc(handles.shiftedImage(:,:,handles.calImageID));
colormap('gray');
daspect([1 1 1]);

% update image ID display
set(handles.CalImageID_text, 'String', [num2str(handles.calImageID) ' of ' num2str(handles.imageNum)]);

% setup buttondownfcn for shifted image
set(handles.shiftedImage_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% setup target list for both reference image and shifted image
handles.refImageTargetList=zeros(handles.imageNum,2);
handles.shiftedImageTargetList=zeros(handles.imageNum,2);

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


% --- Executes on button press in StartAcquisition_button.
function StartAcquisition_button_Callback(hObject, eventdata, handles)
% hObject    handle to StartAcquisition_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% set up target points
% 9 zones calibration, each zone has 4 points 

scanZoom=str2double(get(handles.ScanZoom_edit,'String'));
handles.currentFocus=str2double(get(handles.CurrentFocus_edit,'String'));
handles.currentFocus=handles.currentFocus*1e-6;  % convert to [m]
 
[px py]=meshgrid(-80:32:80, -80:32:80);
xyp=[px(:) py(:)];
theta=-14/180*pi;

%%%%%%%%% for paper
% [px py]=meshgrid(-90:15:90, -90:15:90);
% xyp=[px(:) py(:)];
% theta=-10.5/180*pi;
%%%%%%%%%

xyp=xyp';
for idx=1:size(xyp,2)
    xyp(:,idx)=[cos(theta) -sin(theta); sin(theta) cos(theta)]*xyp(:,idx);
end
xyp=xyp';

xyp=xyp+repmat(handles.SLMPreset,[size(xyp,1) 1]);

xyp=xyp/scanZoom;
xyzp=zeros(size(xyp,1),3);
xyzp(:,1:2)=xyp;
xyzp(:,3)=handles.currentFocus;
handles.xyzp=xyzp;

if ~isempty(handles.f_SLMFocusCalFun)
    handles.objectiveNA=handles.f_SLMFocusCalFun(handles.currentFocus);
end

% update acqImageID
handles.imageNum=size(xyzp,1);
set(handles.AcqImageID_text, 'String', [num2str(handles.acqImageID) ' of ' num2str(handles.imageNum)]);

% start SLM actuation
weight=[1];
automatics=get(handles.Automatics_checkbox, 'Value');
framePeriod=str2double(get(handles.FramePeriod_edit,'String'));
if automatics==1
% automatic acquision, start NI instrument and actuate SLM
    session = daq.createSession ('ni');
    session.addDigitalChannel('dev1','Port0/Line0','OutputOnly');
    session.outputSingleScan(0);
    waitfor(msgbox(['Please set up Prairie to acquire ' num2str(size(xyzp,1)) ' images, and connect D0 port in NI box to Prairie input trigger. Start Prairie Acquisition. Press OK when ready.']));
    for idx=1:size(xyzp,1)
        handles.acqImageID=idx;
        f_SLMActivation_Calibration( handles.SLM_handles, handles.xyzp(idx,:), weight, handles.objectiveNA );
        session.outputSingleScan(1);
        pause(0.1);
        session.outputSingleScan(0);
        set(handles.AcqImageID_text, 'String', [num2str(handles.acqImageID) ' of ' num2str(handles.imageNum)]);
        pause(framePeriod);
    end
    handles.acqImageID=0;
    weight=1;
    f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset handles.currentFocus], weight, handles.objectiveNA );     
else
% manual acquision
    handles.acqImageID=1;
    set(handles.AcqImageID_text, 'String', [num2str(handles.acqImageID) ' of ' num2str(handles.imageNum)]);
    f_SLMActivation_Calibration( handles.SLM_handles, handles.xyzp(handles.acqImageID,:), weight, handles.objectiveNA );     
end
guidata(hObject, handles);


% --- Executes on button press in ActivationPrevious_button.
function ActivationPrevious_button_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationPrevious_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.acqImageID>1
% acutate SLM
    weight=1;
    handles.acqImageID=handles.acqImageID-1;
    f_SLMActivation_Calibration( handles.SLM_handles, handles.xyzp(handles.acqImageID,:), weight, handles.objectiveNA );     
% update acqImageID
    set(handles.AcqImageID_text, 'String', [num2str(handles.acqImageID) ' of ' num2str(handles.imageNum)]);
end
guidata(hObject, handles);


% --- Executes on button press in ActivationNext_button.
function ActivationNext_button_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationNext_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.acqImageID<handles.imageNum && handles.acqImageID>0
% acutate SLM
    weight=1;
    handles.acqImageID=handles.acqImageID+1;
    f_SLMActivation_Calibration( handles.SLM_handles, handles.xyzp(handles.acqImageID,:), weight, handles.objectiveNA );     
% update acqImageID
    set(handles.AcqImageID_text, 'String', [num2str(handles.acqImageID) ' of ' num2str(handles.imageNum)]);
end
guidata(hObject, handles);    


% --- Executes on button press in ActivationFinish_button.
function ActivationFinish_button_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationFinish_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
weight=1;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset handles.currentFocus], weight, handles.objectiveNA );     
handles.acqImageID=0;
guidata(hObject, handles);    


% --- Executes on button press in Automatics_checkbox.
function Automatics_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to Automatics_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Automatics_checkbox

% --- Executes on button press in FinishCalibration_button.
function FinishCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to FinishCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% calculate the transformation matrix
 xy=handles.shiftedImageTargetList-handles.refImageTargetList;
 xy(:,1)=xy(:,1)+size(handles.refImage,2)/2;   % note: x, y is swapped
 xy(:,2)=xy(:,2)+size(handles.refImage,1)/2;   % note: x, y is swapped
 
 % Matrix transformation, from xyp to xy
 xyp=handles.xyzp(:,1:2);
 % construct the 25 zones
 zone=[1 2 7 8]; 
 for idx=1:25
     zoneIndex=zone+(idx-1)+floor((idx-1)/5);
%      [dump,tempIdx]=ismember(21,zoneIndex);
%      if dump==1
%          zoneIndex(tempIdx)=[];
%      end
     tempT(idx)=cp2tform(xyp(zoneIndex,:), xy(zoneIndex,:), 'affine');
 end
 
 idx=1;
 zoneIndex=zone+(idx-1)+floor((idx-1)/5);
 handles.tActivationLaserSLM=cp2tform(xyp(zoneIndex,:), xy(zoneIndex,:), 'affine');
 handles.tActivationLaserSLM=tempT;
 
 % get current focus plane
 handles.currentFocus=str2double(get(handles.CurrentFocus_edit,'String'));
 handles.currentFocus=handles.currentFocus*1e-6;   % convert to [m]
 
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
tActivationLaserSLM=handles.tActivationLaserSLM;
calImageSize=size(handles.refImage);
focusPlane=handles.currentFocus;

shiftedImageTargetList=handles.shiftedImageTargetList;
refImageTargetList=handles.refImageTargetList;

save([path file],'tActivationLaserSLM','calImageSize','focusPlane','shiftedImageTargetList','refImageTargetList');
if isempty(handles.refImageTargetList)
    msgbox('Calibration file not saved. Please press Start Acquision and Finish button. Then press Finish Calibration button and then Save the calibration file again.', 'Error');
end
msgbox('Calibration file Saved. If Current Focus Plane / Scan Zoom has not yet been updated, please update and save again.','Confirm');


% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hf=findobj('Name','CalibrationControl_LateralCalibration');
delete(hf);


% --- Executes on button press in CalibrationPrevious_button.
function CalibrationPrevious_button_Callback(hObject, eventdata, handles)
% hObject    handle to CalibrationPrevious_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calImageID>1
    handles.calImageID=handles.calImageID-1;
    
    axes(handles.ShiftedImage_axe);
    handles.shiftedImage_imageHandle=imagesc(handles.shiftedImage(:,:,handles.calImageID));
    colormap('gray');
    daspect([1 1 1]);
% setup buttondownfcn for shifted image
    set(handles.shiftedImage_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% update calImageID
    set(handles.CalImageID_text, 'String', [num2str(handles.calImageID) ' of ' num2str(handles.imageNum)]);
    handles.shiftedImageTarget_imageHandle=[];

% plot target points for shifted image
    if handles.shiftedImageTargetList(handles.calImageID,1)>0
        axes(handles.ShiftedImage_axe);
        hold on;
        handles.shiftedImageTarget_imageHandle=plot(handles.shiftedImageTargetList(handles.calImageID,1),handles.shiftedImageTargetList(handles.calImageID,2),'ro');
    end
    
% Zoom into the plot
handles.autoZoom=get(handles.AutoZoom_checkbox, 'Value');
if handles.autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.calImageID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.calImageID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ShiftedImage_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ShiftedImage_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(1)]);   
    zoom(gcf,'reset');
end
           
% plot target points for reference image
    if handles.refImageTargetList(handles.calImageID,1)>0
        delete(handles.refImageTarget_imageHandle);
        axes(handles.ReferenceImage_axe);
        hold on;
        handles.refImageTarget_imageHandle=plot(handles.refImageTargetList(handles.calImageID,1),handles.refImageTargetList(handles.calImageID,2),'ro');
    else
        handles.refImageTargetList(handles.calImageID,:)=handles.refImageTargetList(handles.calImageID+1,:);
    end

    guidata(hObject, handles);
end


% --- Executes on button press in CalibrationNext_button.
function CalibrationNext_button_Callback(hObject, eventdata, handles)
% hObject    handle to CalibrationNext_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.calImageID<handles.imageNum
    handles.calImageID=handles.calImageID+1;
    
    axes(handles.ShiftedImage_axe);
    handles.shiftedImage_imageHandle=imagesc(handles.shiftedImage(:,:,handles.calImageID));
    colormap('gray');
    daspect([1 1 1]);
% setup buttondownfcn for shifted image
    set(handles.shiftedImage_imageHandle,'ButtonDownFcn',{@ImageClickCallback,hObject,handles});

% update calImageID
    set(handles.CalImageID_text, 'String', [num2str(handles.calImageID) ' of ' num2str(handles.imageNum)]);
    handles.shiftedImageTarget_imageHandle=[];
    
% plot target points for shifted image
    if handles.shiftedImageTargetList(handles.calImageID,1)>0
        axes(handles.ShiftedImage_axe);
        hold on;
        handles.shiftedImageTarget_imageHandle=plot(handles.shiftedImageTargetList(handles.calImageID,1),handles.shiftedImageTargetList(handles.calImageID,2),'ro');
    end

% Zoom into the plot
handles.autoZoom=get(handles.AutoZoom_checkbox, 'Value');
if handles.autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.calImageID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.calImageID-1)/6)+1)-handles.imageSize(1)/12;    
    axes(handles.ShiftedImage_axe);
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ShiftedImage_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(1)]);   
    zoom(gcf,'reset');
end

% plot target points for reference image
    if handles.refImageTargetList(handles.calImageID,1)>0
        delete(handles.refImageTarget_imageHandle);
        axes(handles.ReferenceImage_axe);
        hold on;
        handles.refImageTarget_imageHandle=plot(handles.refImageTargetList(handles.calImageID,1),handles.refImageTargetList(handles.calImageID,2),'ro');
    else
        handles.refImageTargetList(handles.calImageID,:)=handles.refImageTargetList(handles.calImageID-1,:);
    end
        
guidata(hObject, handles);
end


% --- Executes during object creation, after setting all properties.
function ReferenceImage_axe_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ReferenceImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object deletion, before destroying properties.
function ReferenceImage_axe_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to ReferenceImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function ReferenceImage_axe_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to ReferenceImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function ShiftedImage_axe_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ShiftedImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on mouse press over axes background.
function ShiftedImage_axe_ButtonDownFcn(hObject, ~, handles)
% hObject    handle to ShiftedImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object deletion, before destroying properties.
function ShiftedImage_axe_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to ShiftedImage_axe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function ImageClickCallback ( objectHandle , eventData, hObject, handles )
   handles = guidata(hObject);

   if isempty(handles.referenceImage_imageHandle) || isempty(handles.shiftedImage_imageHandle)
       return;
   end
   
   if objectHandle==handles.referenceImage_imageHandle
       axesHandle  = get(objectHandle,'Parent');
       coordinates = get(axesHandle,'CurrentPoint'); 
       x = coordinates(1,1);
       y = coordinates(1,2);

       if isempty(handles.refImageTarget_imageHandle)
           hold on;
           handles.refImageTarget_imageHandle=plot(x,y,'ro');
       else
           delete(handles.refImageTarget_imageHandle);
           hold on;
           handles.refImageTarget_imageHandle=plot(x,y,'ro');
       end           
       handles.refImageTargetList(handles.calImageID,:)=[x y];

   else if objectHandle==handles.shiftedImage_imageHandle 
       axesHandle  = get(objectHandle,'Parent');
       coordinates = get(axesHandle,'CurrentPoint'); 
       x = coordinates(1,1);
       y = coordinates(1,2);
       if isempty(handles.shiftedImageTarget_imageHandle)
           hold on;
           handles.shiftedImageTarget_imageHandle=plot(x,y,'ro');
       else
           delete(handles.shiftedImageTarget_imageHandle);
           hold on;
           handles.shiftedImageTarget_imageHandle=plot(x,y,'ro');
       end    
       handles.shiftedImageTargetList(handles.calImageID,:)=[x y];
       end
   end
    guidata(hObject, handles);



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


% --- Executes on button press in SelectCalibrationFile_button.
function SelectCalibrationFile_button_Callback(hObject, eventdata, handles)
% hObject    handle to SelectCalibrationFile_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No axial calibration file is selected.','error');
    return;
end

load([pathName fileName]);
handles.f_SLMFocusCalFun=f_SLMFocusCalFun;
guidata(hObject, handles);



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


% --- Executes on button press in AutoZoom_checkbox.
function AutoZoom_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to AutoZoom_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of AutoZoom_checkbox


% --- Executes on button press in AutoFind_pushbutton.
function AutoFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to AutoFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

interX=handles.imageSize(1)/6;
interY=handles.imageSize(1)/6;
zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.calImageID-1,6)+1)+handles.imageSize(1)/12;
zoneY=handles.imageSize(1)/6*(floor((handles.calImageID-1)/6)+1)-handles.imageSize(1)/12;    

if handles.shiftedImageTargetList(handles.calImageID,1)>0
    x1=handles.shiftedImageTargetList(handles.calImageID,1)-interX*0.2;
    y1=handles.shiftedImageTargetList(handles.calImageID,2)-interY*0.2;
    x2=handles.shiftedImageTargetList(handles.calImageID,1)+interX*0.2;
    y2=handles.shiftedImageTargetList(handles.calImageID,2)+interY*0.2;
else
    x1=zoneX-interX*0.5;
    y1=zoneY-interY*0.5;
    x2=zoneX+interX*0.5;
    y2=zoneY+interY*0.5;    
end
    
if x1<=0 x1=1; end
if y1<=0 y1=1; end
if x2>handles.imageSize(1) x2=handles.imageSize(1); end
if y2>handles.imageSize(1) y2=handles.imageSize(1); end

x1=ceil(x1);
y1=ceil(y1);
x2=floor(x2);
y2=floor(y2);

data=double(handles.shiftedImage(y1:y2,x1:x2,handles.calImageID));
[xymax,smax,xymin,smin] = extrema2(data);
[xmin,ymin]=ind2sub([y2-y1+1 x2-x1+1],smin(1));
x=ymin+x1-1;
y=xmin+y1-1;

handles.shiftedImageTargetList(handles.calImageID,:)=[x y];

% plot target points for shifted image
axes(handles.ShiftedImage_axe);    
if isempty(handles.shiftedImageTarget_imageHandle)
   hold on;
   handles.shiftedImageTarget_imageHandle=plot(x,y,'ro');
else
   delete(handles.shiftedImageTarget_imageHandle);
   hold on;
   handles.shiftedImageTarget_imageHandle=plot(x,y,'ro');
end    

% Zoom into the plot
handles.autoZoom=get(handles.AutoZoom_checkbox, 'Value');
if handles.autoZoom==1
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ShiftedImage_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(1)]);   
    zoom(gcf,'reset');
end

guidata(hObject, handles);


% --- Executes on button press in CenterMassFind_pushbutton.
function CenterMassFind_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to CenterMassFind_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.shiftedImageTargetList(handles.calImageID,1)==0
    return;
end

x0=round(handles.shiftedImageTargetList(handles.calImageID,1));
y0=round(handles.shiftedImageTargetList(handles.calImageID,2));

centerOfMassRange=str2num(get(handles.CenterOfMassRange_edit,'String'));
centerOfMassHalfRange=floor(centerOfMassRange/2);
centerOfMassThreshold=str2num(get(handles.CenterOfMassThreshold_edit,'String'));

[tempX tempY]=meshgrid(x0-centerOfMassHalfRange:x0+centerOfMassHalfRange, ...
    y0-centerOfMassHalfRange:y0+centerOfMassHalfRange);

data=handles.shiftedImage(y0-centerOfMassHalfRange:y0+centerOfMassHalfRange, ...
    x0-centerOfMassHalfRange:x0+centerOfMassHalfRange,handles.calImageID);
data=double(data);
index=find(data<(mean(data(:))-centerOfMassThreshold*std(data(:))));
if isempty(index)
    return;
end

xc=sum(tempX(index).*data(index))/sum(data(index));
yc=sum(tempY(index).*data(index))/sum(data(index));

handles.shiftedImageTargetList(handles.calImageID,:)=[xc yc];

% plot target points for shifted image
axes(handles.ShiftedImage_axe);    
if isempty(handles.shiftedImageTarget_imageHandle)
   hold on;
   handles.shiftedImageTarget_imageHandle=plot(xc,yc,'ro');
else
   delete(handles.shiftedImageTarget_imageHandle);
   hold on;
   handles.shiftedImageTarget_imageHandle=plot(xc,yc,'ro');
end    

% Zoom into the plot
handles.autoZoom=get(handles.AutoZoom_checkbox, 'Value');
if handles.autoZoom==1
    zoneX=handles.imageSize(1)-handles.imageSize(1)/6*(mod(handles.calImageID-1,6)+1)+handles.imageSize(1)/12;
    zoneY=handles.imageSize(1)/6*(floor((handles.calImageID-1)/6)+1)-handles.imageSize(1)/12;    
    axis([zoneX-handles.imageSize(1)/6 zoneX+handles.imageSize(1)/6 zoneY-handles.imageSize(1)/6 zoneY+handles.imageSize(1)/6]);
else
    axes(handles.ShiftedImage_axe);
    axis([1 handles.imageSize(1) 1 handles.imageSize(1)]);   
    zoom(gcf,'reset');
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
