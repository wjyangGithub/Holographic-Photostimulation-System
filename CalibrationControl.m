% Calibration control menu: to initiate various calibrations and assemble
% calibration files
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationControl(varargin)
%CALIBRATIONCONTROL M-file for CalibrationControl.fig
%      CALIBRATIONCONTROL, by itself, creates a new CALIBRATIONCONTROL or raises the existing
%      singleton*.
%
%      H = CALIBRATIONCONTROL returns the handle to a new CALIBRATIONCONTROL or the handle to
%      the existing singleton*.
%
%      CALIBRATIONCONTROL('Property','Value',...) creates a new CALIBRATIONCONTROL using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to CalibrationControl_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CALIBRATIONCONTROL('CALLBACK') and CALIBRATIONCONTROL('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CALIBRATIONCONTROL.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationControl

% Last Modified by GUIDE v2.5 21-May-2017 17:18:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationControl_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationControl_OutputFcn, ...
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


% --- Executes just before CalibrationControl is made visible.
function CalibrationControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for CalibrationControl
handles.output = hObject;

% self defined parameters
handles.SLM_handles=varargin{1};                  % SLM handle
handles.f_SLMFocusCalFun=[];                      % fitted function of focal depth vs. effective NA
handles.tActivationLaserSLM=[];                   % lateral calibration matrix
handles.tActivationLaserSLM_calImageSize=[];      % lateral calibration image size
handles.tActivationLaserSLM_focusPlane=[];        % lateral calibration focusPlane
handles.tActivationLaserSLM_focusPlaneNumber=0;   % lateral calibration, total focus plane loaded
handles.tImage2Activation=[];                     % two laser mapping calibration matrix
handles.tImage2Activation_focusPlane=[];          % two laser mapping calibration focusPlane
handles.tImage2Activation_focusPlaneNumber=0;     % two laser mapping calibration, total focus plane loaded
handles.tImage2Activation_imageSize=[];           % two laser mapping imaging laser image size
handles.tImage2Activation_activationSize=[];      % two laser mapping activation laser image size
handles.XYWeightCorrection=[];                    % XY weight correction (X,Y SLM coordinate and correction weight) 
handles.ZWeightCorrection=[];                     % Z weight correction (Z[m] coordinate and correction weight)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationControl wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationControl_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function AxialCalibrationFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to AxialCalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AxialCalibrationFile_edit as text
%        str2double(get(hObject,'String')) returns contents of AxialCalibrationFile_edit as a double


% --- Executes during object creation, after setting all properties.
function AxialCalibrationFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AxialCalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseAxialCalibration_button.
function BrowseAxialCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseAxialCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.AxialCalibrationFile_edit, 'String', [pathName fileName]);
load([pathName fileName]);
handles.f_SLMFocusCalFun=f_SLMFocusCalFun;
guidata(hObject, handles);



function LateralCalibrationFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to LateralCalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LateralCalibrationFile_edit as text
%        str2double(get(hObject,'String')) returns contents of LateralCalibrationFile_edit as a double



% --- Executes during object creation, after setting all properties.
function LateralCalibrationFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LateralCalibrationFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseLateralCalibration_button.
function BrowseLateralCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseLateralCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.LateralCalibrationFile_edit, 'String', [pathName fileName]);
load([pathName fileName]);

if isempty(find(handles.tActivationLaserSLM_focusPlane==focusPlane))
    handles.tActivationLaserSLM_focusPlaneNumber=handles.tActivationLaserSLM_focusPlaneNumber+1;
    handles.tActivationLaserSLM_focusPlane=[handles.tActivationLaserSLM_focusPlane; focusPlane];
    handles.tActivationLaserSLM_calImageSize=[handles.tActivationLaserSLM_calImageSize; calImageSize];
    handles.tActivationLaserSLM{handles.tActivationLaserSLM_focusPlaneNumber}=tActivationLaserSLM;                   
    
    set(handles.LateralCalibration_table,'Data',[handles.tActivationLaserSLM_focusPlane*1e6 ones(handles.tActivationLaserSLM_focusPlaneNumber,1)]);
end

guidata(hObject, handles);


function LaserMappingFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to LaserMappingFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LaserMappingFile_edit as text
%        str2double(get(hObject,'String')) returns contents of LaserMappingFile_edit as a double


% --- Executes during object creation, after setting all properties.
function LaserMappingFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LaserMappingFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BrowseLaserMapping_button.
function BrowseLaserMapping_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseLaserMapping_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.LaserMappingFile_edit, 'String', [pathName fileName]);
load([pathName fileName]);

if isempty(find(handles.tImage2Activation_focusPlane==focusPlane))
    handles.tImage2Activation_focusPlaneNumber=handles.tImage2Activation_focusPlaneNumber+1;
    handles.tImage2Activation_focusPlane=[handles.tImage2Activation_focusPlane; focusPlane];
    handles.tImage2Activation{handles.tImage2Activation_focusPlaneNumber}=tImage2Activation;                   
        
    handles.tImage2Activation_imageSize=imageSize;
    handles.tImage2Activation_activationSize=activationSize;
    
    set(handles.LaserMapping_table,'Data',[handles.tImage2Activation_focusPlane*1e6 ones(handles.tImage2Activation_focusPlaneNumber,1)]);
end

guidata(hObject, handles);

% --- Executes on button press in BrowseAxialWeight_button.
function BrowseAxialWeight_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseAxialWeight_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.AxialWeightFile_edit, 'String', [pathName fileName]);
load([pathName fileName]);

handles.ZWeightCorrection=ZWeightCorrection;    % correction weight (E field) for Z
guidata(hObject, handles);

% --- Executes on button press in BrowseLateralWeight_button.
function BrowseLateralWeight_button_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseLateralWeight_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No calibration file is selected.','error');
    return;
end
set(handles.LateralWeightFile_edit, 'String', [pathName fileName]);
load([pathName fileName]);

handles.XYWeightCorrection=XYWeightCorrection;      % correction weight (E field) for XY
guidata(hObject, handles);

% --- Executes on button press in NewAxialCalibration_button.
function NewAxialCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to NewAxialCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationControl_AxialCalibration(handles.SLM_handles);


% --- Executes on button press in NewLateralCalibration_button.
function NewLateralCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to NewLateralCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationControl_LateralCalibration(handles.SLM_handles);

% --- Executes on button press in NewLaserMappingCalibration.
function NewLaserMappingCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to NewLaserMappingCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationControl_TwoLaserMapping(handles.SLM_handles);

% --- Executes on button press in SaveCalibration_button.
function SaveCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to SaveCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save Calibration File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration file cannot be saved', 'Error');
    return;
end

f_SLMFocusCalFun=handles.f_SLMFocusCalFun;

[tActivationLaserSLM_focusPlane,index]=sort(handles.tActivationLaserSLM_focusPlane);
tActivationLaserSLM_calImageSize=handles.tActivationLaserSLM_calImageSize(index,:);
for idx=1:handles.tActivationLaserSLM_focusPlaneNumber
    tActivationLaserSLM{idx}=handles.tActivationLaserSLM{index(idx)};
end

[tImage2Activation_focusPlane,index]=sort(handles.tImage2Activation_focusPlane);
for idx=1:handles.tImage2Activation_focusPlaneNumber
    tImage2Activation{idx}=handles.tImage2Activation{index(idx)};
end

XYWeightCorrection=handles.XYWeightCorrection;
ZWeightCorrection=handles.ZWeightCorrection;

save([path file],'f_SLMFocusCalFun','tActivationLaserSLM','tActivationLaserSLM_focusPlane','tActivationLaserSLM_calImageSize','tImage2Activation','tImage2Activation_focusPlane', ...
    'XYWeightCorrection', 'ZWeightCorrection');


% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

hf=findobj('Name','CalibrationControl');
delete(hf);


% --- Executes on button press in SaveCalibrationGalvotoResonance_pushbutton.
function SaveCalibrationGalvotoResonance_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to SaveCalibrationGalvotoResonance_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[file,path] = uiputfile('*.mat','Save Galvo to Resonance Calibration File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration file cannot be saved', 'Error');
    return;
end

[tGalvo2Resonance_focusPlane,index]=sort(handles.tImage2Activation_focusPlane);
for idx=1:handles.tImage2Activation_focusPlaneNumber
    tGalvo2Resonance{idx}=handles.tImage2Activation{index(idx)};
end

tGalvo2Resonance_galvoImageSize=handles.tImage2Activation_imageSize;
tGalvo2Resonance_resonanceImageSize=handles.tImage2Activation_activationSize;

save([path file],'tGalvo2Resonance','tGalvo2Resonance_focusPlane','tGalvo2Resonance_galvoImageSize','tGalvo2Resonance_resonanceImageSize');



function AxialWeightFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to AxialWeightFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AxialWeightFile_edit as text
%        str2double(get(hObject,'String')) returns contents of AxialWeightFile_edit as a double


% --- Executes during object creation, after setting all properties.
function AxialWeightFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AxialWeightFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function LateralWeightFile_edit_Callback(hObject, eventdata, handles)
% hObject    handle to LateralWeightFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LateralWeightFile_edit as text
%        str2double(get(hObject,'String')) returns contents of LateralWeightFile_edit as a double


% --- Executes during object creation, after setting all properties.
function LateralWeightFile_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LateralWeightFile_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in NewAxialWeightCalibration_button.
function NewAxialWeightCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to NewAxialWeightCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox(['Please measure the two-photon fluoresence intensity I with different SLM defocusing (negative means longer focal length).' ...
    'Create a .mat file with a variable named ZWeightCorrection. ZWeightCorrection.Z is the defocusing length in meter, and ZWeightCorrection.Mat is I.^-0.25']);

% --- Executes on button press in NewLateralWeightCalibration_button.
function NewLateralWeightCalibration_button_Callback(hObject, eventdata, handles)
% hObject    handle to NewLateralWeightCalibration_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationControl_LateralWeightCalibration(handles.SLM_handles);
