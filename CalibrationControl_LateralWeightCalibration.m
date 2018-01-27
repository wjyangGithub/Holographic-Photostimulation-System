% Lateral weight calibration: to calibrate the weighting factor for different
% lateral deflection by SLM
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationControl_LateralWeightCalibration(varargin)
% CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION MATLAB code for CalibrationControl_LateralWeightCalibration.fig
%      CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION, by itself, creates a new CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION or raises the existing
%      singleton*.
%
%      H = CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION returns the handle to a new CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION or the handle to
%      the existing singleton*.
%
%      CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION.M with the given input arguments.
%
%      CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION('Property','Value',...) creates a new CALIBRATIONCONTROL_LATERALWEIGHTCALIBRATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CalibrationControl_LateralWeightCalibration_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CalibrationControl_LateralWeightCalibration_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationControl_LateralWeightCalibration

% Last Modified by GUIDE v2.5 21-May-2017 21:35:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationControl_LateralWeightCalibration_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationControl_LateralWeightCalibration_OutputFcn, ...
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


% --- Executes just before CalibrationControl_LateralWeightCalibration is made visible.
function CalibrationControl_LateralWeightCalibration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CalibrationControl_LateralWeightCalibration (see VARARGIN)

% Choose default command line output for CalibrationControl_LateralWeightCalibration
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Self defined parameters
handles.SLM_handles=varargin{1};    % SLM handle
CFG;
handles.SLMPreset=SLMPreset;
handles.objectiveNA=0.6;
handles.acqImageID=0;

handles.image=[];
handles.imageSize=[];
handles.image_imageHandle=[];

handles.currentFocus=0;
handles.f_SLMFocusCalFun=[];

handles.autoZoom=0;

handles.gridPlot=[];
handles.X=[];
handles.Y=[];

handles.intensity2P=[];
handles.correctionMat=[];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationControl_LateralWeightCalibration wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationControl_LateralWeightCalibration_OutputFcn(hObject, eventdata, handles) 
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


% --- Executes on button press in BrowseImageStack_button.
function BrowseImageStack_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseImageStack_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.tif');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No image is selected.','error');
    return;
end
set(handles.ImageStackFile_edit, 'String', [pathName fileName]);
handles.image=f_readTiffFile([pathName fileName]);
handles.imageSize=[size(handles.image,1) size(handles.image,2)];

% plot image file (maximum projected)
axes(handles.Image_axes);
handles.image_imageHandle=imagesc(max(handles.image,[],3));
colormap('gray');
daspect([1 1 1]);

guidata(hObject, handles);



function FirstTargetCentroidX_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FirstTargetCentroidX_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FirstTargetCentroidX_edit as text
%        str2double(get(hObject,'String')) returns contents of FirstTargetCentroidX_edit as a double


% --- Executes during object creation, after setting all properties.
function FirstTargetCentroidX_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FirstTargetCentroidX_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function XStep_edit_Callback(hObject, eventdata, handles)
% hObject    handle to XStep_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of XStep_edit as text
%        str2double(get(hObject,'String')) returns contents of XStep_edit as a double


% --- Executes during object creation, after setting all properties.
function XStep_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XStep_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FirstTargetCentroidY_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FirstTargetCentroidY_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FirstTargetCentroidY_edit as text
%        str2double(get(hObject,'String')) returns contents of FirstTargetCentroidY_edit as a double


% --- Executes during object creation, after setting all properties.
function FirstTargetCentroidY_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FirstTargetCentroidY_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YStep_edit_Callback(hObject, eventdata, handles)
% hObject    handle to YStep_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YStep_edit as text
%        str2double(get(hObject,'String')) returns contents of YStep_edit as a double


% --- Executes during object creation, after setting all properties.
function YStep_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YStep_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Diameter_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Diameter_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Diameter_edit as text
%        str2double(get(hObject,'String')) returns contents of Diameter_edit as a double


% --- Executes during object creation, after setting all properties.
function Diameter_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Diameter_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in PlotGrid_button.
function PlotGrid_button_Callback(hObject, eventdata, handles)
% hObject    handle to PlotGrid_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.gridPlot)
    delete(handles.gridPlot);
end
    
x0=str2double(get(handles.FirstTargetCentroidX_edit,'String'));
y0=str2double(get(handles.FirstTargetCentroidY_edit,'String'));
deltax=str2double(get(handles.XStep_edit,'String'));
deltay=str2double(get(handles.YStep_edit,'String'));

[X Y]=meshgrid(x0:-deltax:(x0-12*deltax),y0:deltay:(y0+12*deltay));
X=transpose(X);
Y=transpose(Y);
axes(handles.Image_axes);
hold on;
handles.gridPlot=plot(X(:),Y(:),'ro');
handles.X=X;
handles.Y=Y;
hold off;

guidata(hObject, handles);


% --- Executes on button press in Calculate_button.
function Calculate_button_Callback(hObject, eventdata, handles)
% hObject    handle to Calculate_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
intensity2P=zeros(169,1);
diameter=str2double(get(handles.Diameter_edit,'String'));
background=str2double(get(handles.Background_edit,'String'));
[imageX imageY]=meshgrid(1:handles.imageSize(1),1:handles.imageSize(2));

for idx=1:169
    index=find(((imageX(:)-handles.X(idx)).^2+(imageY(:)-handles.Y(idx)).^2)<(diameter/2).^2);
    temp=double(handles.image(:,:,idx))-background;
    intensity2P(idx)=sum(temp(index));
end
intensity2P=intensity2P./max(intensity2P);

axes(handles.Weight_axes);
contourf(handles.X,handles.Y,reshape(intensity2P,13,13),0:0.05:1,'linestyle','none');
set(gca,'Ydir','reverse');
handles.intensity2P=abs(intensity2P);
handles.correctionMat=intensity2P.^(-0.25);

guidata(hObject, handles);


% --- Executes on button press in Save_button.
function Save_button_Callback(hObject, eventdata, handles)
% hObject    handle to Save_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save Lateral Calibration File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration file cannot be saved', 'Error');
    return;
end

[XYWeightCorrection.X XYWeightCorrection.Y]=meshgrid(-120:5:120, -120:5:120);
XYWeightCorrection.Mat=griddata(handles.xyzp(:,1),handles.xyzp(:,2),handles.correctionMat, XYWeightCorrection.X, XYWeightCorrection.Y);

save([path file],'XYWeightCorrection');
msgbox('Calibration file Saved.','Confirm');


function Background_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Background_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Background_edit as text
%        str2double(get(hObject,'String')) returns contents of Background_edit as a double


% --- Executes during object creation, after setting all properties.
function Background_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Background_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


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


% --- Executes on button press in Automatics_checkbox.
function Automatics_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to Automatics_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Automatics_checkbox



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
scanZoom=str2double(get(handles.ScanZoom_edit,'String'));
handles.currentFocus=str2double(get(handles.CurrentFocus_edit,'String'));
handles.currentFocus=handles.currentFocus*1e-6;  % convert to [m]
 
[px py]=meshgrid(-90:15:90, -90:15:90);
xyp=[px(:) py(:)];
theta=-15/180*pi;

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


% --- Executes on button press in ActivationFinish_button.
function ActivationFinish_button_Callback(hObject, eventdata, handles)
% hObject    handle to ActivationFinish_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
weight=1;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset handles.currentFocus], weight, handles.objectiveNA );     
handles.acqImageID=0;
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
