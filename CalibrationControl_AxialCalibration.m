% Axial calibration: to calibrate the axial shift by SLM
% Author: Weijian Yang, 2015-2018

function varargout = CalibrationControl_AxialCalibration(varargin)
%CALIBRATIONCONTROL_AXIALCALIBRATION M-file for CalibrationControl_AxialCalibration.fig
%      CALIBRATIONCONTROL_AXIALCALIBRATION, by itself, creates a new CALIBRATIONCONTROL_AXIALCALIBRATION or raises the existing
%      singleton*.
%
%      H = CALIBRATIONCONTROL_AXIALCALIBRATION returns the handle to a new CALIBRATIONCONTROL_AXIALCALIBRATION or the handle to
%      the existing singleton*.
%
%      CALIBRATIONCONTROL_AXIALCALIBRATION('Property','Value',...) creates a new CALIBRATIONCONTROL_AXIALCALIBRATION using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to CalibrationControl_AxialCalibration_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CALIBRATIONCONTROL_AXIALCALIBRATION('CALLBACK') and CALIBRATIONCONTROL_AXIALCALIBRATION('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CALIBRATIONCONTROL_AXIALCALIBRATION.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationControl_AxialCalibration

% Last Modified by GUIDE v2.5 15-Mar-2015 20:56:02

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationControl_AxialCalibration_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationControl_AxialCalibration_OutputFcn, ...
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


% --- Executes just before CalibrationControl_AxialCalibration is made visible.
function CalibrationControl_AxialCalibration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for CalibrationControl_AxialCalibration
handles.output = hObject;

% Self defined parameters
CFG;
handles.SLMPreset=SLMPreset;
handles.SLM_handles=varargin{1};    % SLM handle

handles.planeID=0;                  % current plane ID
handles.planeStack=[];              % focal plane to be scanned by SLM
handles.NAStack=[];                 % effective NA of each plane
handles.f_SLMFocusCalFun=[];        % fitted function of focal depth vs. effective NA

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationControl_AxialCalibration wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationControl_AxialCalibration_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function SLMPlaneInterval_edit_Callback(hObject, eventdata, handles)
% hObject    handle to SLMPlaneInterval_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SLMPlaneInterval_edit as text
%        str2double(get(hObject,'String')) returns contents of SLMPlaneInterval_edit as a double


% --- Executes during object creation, after setting all properties.
function SLMPlaneInterval_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SLMPlaneInterval_edit (see GCBO)
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



function FocusPlane_edit_Callback(hObject, eventdata, handles)
% hObject    handle to FocusPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FocusPlane_edit as text
%        str2double(get(hObject,'String')) returns contents of FocusPlane_edit as a double


% --- Executes during object creation, after setting all properties.
function FocusPlane_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FocusPlane_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EffectiveNA_edit_Callback(hObject, eventdata, handles)
% hObject    handle to EffectiveNA_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EffectiveNA_edit as text
%        str2double(get(hObject,'String')) returns contents of EffectiveNA_edit as a double

guidata(hObject,handles);



% --- Executes during object creation, after setting all properties.
function EffectiveNA_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EffectiveNA_edit (see GCBO)
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
startPlane=str2double(get(handles.SLMStartPlane_edit, 'String'));
endPlane=str2double(get(handles.SLMEndPlane_edit, 'String'));
planeInterval=str2double(get(handles.SLMPlaneInterval_edit, 'String'));

if isempty(startPlane:planeInterval:endPlane)
    msgbox('Sign of Plane Interval Reversed', 'Error');
    planeInterval=-planeInterval;
    set(handles.SLMPlaneInterval_edit, 'String',num2str(planeInterval));
end

handles.planeStack=(startPlane:planeInterval:endPlane)*1e-6;
handles.NAStack=zeros(1,length(handles.planeStack));
handles.planeID=1;
xyzp=[handles.SLMPreset handles.planeStack(handles.planeID)];
weight=[1];
objectiveNA=str2double(get(handles.EffectiveNA_edit,'String'));
set(handles.FocusPlane_edit, 'String', num2str(handles.planeStack(handles.planeID)*1e6));
f_SLMActivation_Calibration( handles.SLM_handles, xyzp, weight, objectiveNA );
handles.NAStack(handles.planeID)=objectiveNA;

guidata(hObject,handles);
 

% --- Executes on button press in Previous_button.
function Previous_button_Callback(hObject, eventdata, handles)
% hObject    handle to Previous_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.planeID<=1
    return;
else
    handles.planeID=handles.planeID-1;
    set(handles.FocusPlane_edit, 'String', num2str(handles.planeStack(handles.planeID)*1e6));
    if handles.NAStack(handles.planeID)>0
        set(handles.EffectiveNA_edit,'String',num2str(handles.NAStack(handles.planeID)));
        objectiveNA=handles.NAStack(handles.planeID);
    else
        objectiveNA=str2double(get(handles.EffectiveNA_edit,'String'));
        handles.NAStack(handles.planeID)=objectiveNA;
    end    
    xyzp=[handles.SLMPreset handles.planeStack(handles.planeID)];
    weight=1;
    f_SLMActivation_Calibration( handles.SLM_handles, xyzp, weight, objectiveNA );
end
guidata(hObject,handles);

% --- Executes on button press in Next_button.
function Next_button_Callback(hObject, eventdata, handles)
% hObject    handle to Next_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.planeID==length(handles.planeStack) || handles.planeID==0
    return;
else
    handles.planeID=handles.planeID+1;    
    set(handles.FocusPlane_edit, 'String', num2str(handles.planeStack(handles.planeID)*1e6));
    if handles.NAStack(handles.planeID)>0
        set(handles.EffectiveNA_edit,'String',num2str(handles.NAStack(handles.planeID)));
        objectiveNA=handles.NAStack(handles.planeID);
    else
        objectiveNA=str2double(get(handles.EffectiveNA_edit,'String'));
        handles.NAStack(handles.planeID)=objectiveNA;
    end    
    xyzp=[handles.SLMPreset handles.planeStack(handles.planeID)];
    weight=1;
    f_SLMActivation_Calibration( handles.SLM_handles, xyzp, weight, objectiveNA );
end
guidata(hObject,handles);

% --- Executes on button press in Update_button.
function Update_button_Callback(hObject, eventdata, handles)
% hObject    handle to Update_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.planeID==0 || isempty(handles.planeStack)
    return;
end

xyzp=[handles.SLMPreset handles.planeStack(handles.planeID)];
weight=[1];
objectiveNA=str2double(get(handles.EffectiveNA_edit,'String'));
f_SLMActivation_Calibration( handles.SLM_handles, xyzp, weight, objectiveNA );
handles.NAStack(handles.planeID)=objectiveNA;
guidata(hObject,handles);

% --- Executes on button press in FinishAcquision_button.
function FinishAcquision_button_Callback(hObject, eventdata, handles)
% hObject    handle to FinishAcquision_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.planeStack)
    [f_SLMFocusCalFun,gof,out] = fit( handles.planeStack', handles.NAStack', 'linearinterp' );
    handles.f_SLMFocusCalFun=f_SLMFocusCalFun;
    guidata(hObject,handles);
end
weight=1;
objectiveNA=0.4;
f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], weight, objectiveNA );

% --- Executes on button press in Save_button.
function Save_button_Callback(hObject, eventdata, handles)
% hObject    handle to Save_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.f_SLMFocusCalFun)
    msgbox('Please press "Finish Acquision" before saving.', 'Error');
    return;
end

[file,path] = uiputfile('*.mat','Save Axial Calibration File');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Calibration file cannot be saved', 'Error');
    return;
end
f_SLMFocusCalFun=handles.f_SLMFocusCalFun;
save([path file],'f_SLMFocusCalFun');

% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hf=findobj('Name','CalibrationControl_AxialCalibration');
delete(hf);
