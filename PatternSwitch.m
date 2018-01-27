% to switch between different holograms
% Author: Weijian Yang, 2015-2018

function varargout = PatternSwitch(varargin)
% PATTERNSWITCH M-file for PatternSwitch.fig
%      PATTERNSWITCH, by itself, creates a new PATTERNSWITCH or raises the existing
%      singleton*.
%
%      H = PATTERNSWITCH returns the handle to a new PATTERNSWITCH or the handle to
%      the existing singleton*.
%
%      PATTERNSWITCH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PATTERNSWITCH.M with the given input arguments.
%
%      PATTERNSWITCH('Property','Value',...) creates a new PATTERNSWITCH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PatternSwitch_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PatternSwitch_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PatternSwitch

% Last Modified by GUIDE v2.5 11-Jun-2015 10:58:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PatternSwitch_OpeningFcn, ...
                   'gui_OutputFcn',  @PatternSwitch_OutputFcn, ...
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


% --- Executes just before PatternSwitch is made visible.
function PatternSwitch_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PatternSwitch (see VARARGIN)

% Choose default command line output for PatternSwitch
handles.output = hObject;

% self defined parameters
handles.SLM_handles=varargin{1};    % SLM handle
handles.stateNum = 0;               % number of SLM state
handles.SLMPhase = [];              % SLM phase pattern
handles.ItineraryTableData = [];    % itinerary table data
handles.repetition = 10;            % number of repetition

CFG;
handles.SLMPreset=SLMPreset;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PatternSwitch wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PatternSwitch_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in AddState_button.
function AddState_button_Callback(hObject, eventdata, handles)
% hObject    handle to AddState_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No ROI list file is selected.','error');
    return;
end
load([pathName fileName]);

handles.stateNum=handles.stateNum+1;
stateID=handles.stateNum;

handles.SLMPhase{stateID}=SLMPhase;
handles.ROIlistName{stateID}=fileName;
handles.ItineraryTableData(stateID,1)=handles.stateNum;
handles.ItineraryTableData(stateID,2)=500;
handles.ItineraryTableData(stateID,3)=100;
tempData=[handles.ROIlistName' num2cell(handles.ItineraryTableData)];
set(handles.Itinerary_table,'Data',tempData);

guidata(hObject, handles);


% --- Executes on button press in DeleteState_button.
function DeleteState_button_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteState_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

deleteStateID=str2double(get(handles.DeleteStateID_edit, 'String'));
index=find(handles.ItineraryTableData(:,1)==deleteStateID);
if isempty(index)
    msgbox('No such state is found', 'Error');
    return;
else
    handles.stateNum=handles.stateNum-1;
    handles.ItineraryTableData(index,:)=[];
    handles.ROIlistName{index}=[];
    handles.SLMPhase{index}=[];
    handles.ROIlistName=handles.ROIlistName(~cellfun('isempty',handles.ROIlistName));
    handles.SLMPhase=handles.SLMPhase(~cellfun('isempty',handles.SLMPhase));
    
    tempData=[handles.ROIlistName' num2cell(handles.ItineraryTableData)];
    set(handles.Itinerary_table,'Data',tempData);
end

guidata(hObject, handles);


function DeleteStateID_edit_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteStateID_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DeleteStateID_edit as text
%        str2double(get(hObject,'String')) returns contents of DeleteStateID_edit as a double


% --- Executes during object creation, after setting all properties.
function DeleteStateID_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DeleteStateID_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in GenerateExperiment_button.
function GenerateExperiment_button_Callback(hObject, eventdata, handles)
% hObject    handle to GenerateExperiment_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.repetition=str2double(get(handles.Repetition_edit, 'String'));

[~,index] = sort(handles.ItineraryTableData(:,1),'ascend');
handles.ItineraryTableData=handles.ItineraryTableData(index,:);
handles.ROIlistName=handles.ROIlistName(index);
handles.SLMPhase=handles.SLMPhase(index);

tempData=[handles.ROIlistName' num2cell(handles.ItineraryTableData)];
set(handles.Itinerary_table,'Data',tempData);

guidata(hObject, handles);


function Repetition_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Repetition_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Repetition_edit as text
%        str2double(get(hObject,'String')) returns contents of Repetition_edit as a double


% --- Executes during object creation, after setting all properties.
function Repetition_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Repetition_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SaveItinerary_button.
function SaveItinerary_button_Callback(hObject, eventdata, handles)
% hObject    handle to SaveItinerary_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save Itinerary List');
if max(size(file)) == 1 | file == 0
    msgbox('No file name input. Itinerary List file cannot be saved', 'Error');
    return;
end
ItineraryData.SLMPhase = handles.SLMPhase;
ItineraryData.ROIlistName=handles.ROIlistName;
ItineraryData.ItineraryTableData=handles.ItineraryTableData;
ItineraryData.stateNum=handles.stateNum;
ItineraryData.repetition=handles.repetition;

save([path file],'ItineraryData');


% --- Executes on button press in LoadItinerary_button.
function LoadItinerary_button_Callback(hObject, eventdata, handles)
% hObject    handle to LoadItinerary_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName,pathName] = uigetfile('*.mat');                        
if max(size(fileName)) == 1 | fileName == 0
    msgbox('No Itinerary List file is selected.','error');
    return;
end
load([pathName fileName]);
handles.SLMPhase=ItineraryData.SLMPhase;
handles.ROIlistName=ItineraryData.ROIlistName;
handles.ItineraryTableData=ItineraryData.ItineraryTableData;
handles.stateNum=ItineraryData.stateNum;
handles.repetition=ItineraryData.repetition;

tempData=[handles.ROIlistName' num2cell(handles.ItineraryTableData)];
set(handles.Itinerary_table,'Data',tempData);

set(handles.Repetition_edit, 'String', num2str(handles.repetition));

guidata(hObject, handles);


% --- Executes on button press in StartExperiment_button.
function StartExperiment_button_Callback(hObject, eventdata, handles)
% hObject    handle to StartExperiment_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

CFG;

% start NI instrument and actuate SLM
 session = daq.createSession ('ni');
 session.addDigitalChannel('dev1','Port0/Line0','OutputOnly');
 session.outputSingleScan(0);
 waitfor(msgbox(['Please set up Prairie for ' num2str(handles.repetition*handles.stateNum) ' of activation, and connect D0 port in NI box to Prairie input trigger. Set up Prairie activation "Trigger" to be "TrigIn", and "WaitforTrigger" as "EveryRepetition". Start Prairie Acquisition. Press OK when ready.']));
 set(handles.StartExperiment_button,'enable','off');

 for ii=1:handles.repetition
 for idx=1:handles.stateNum    
     if strcmp(SLM, 'BNS')
        BNS_SLM_LoadImage( handles.SLMPhase{idx}, handles.SLM_handles );
     else
        Holoeye_SLM_LoadImage( handles.SLMPhase{idx}, SLM_M, SLM_N, SLMm0, SLMn0, handles.SLM_handles, calibratedLUTFunctionfileName );
     end
     pause(SLMLoadTime);
     session.outputSingleScan(1);
     pause(0.05);
     session.outputSingleScan(0);
     pause(handles.ItineraryTableData(idx,2)/1000);
     pause(handles.ItineraryTableData(idx,3)/1000);     
 end
 end

set(handles.StartExperiment_button,'enable','on');
%f_SLMActivation_Calibration( handles.SLM_handles, [handles.SLMPreset 0], 1, 0.4 );
f_SLMActivation_Calibration( handles.SLM_handles, [0 0 0], 1, 0.4 );


% --- Executes on button press in Exit_button.
function Exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to Exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hf=findobj('Name','PatternSwitch');
delete(hf);


% --- Executes when entered data in editable cell(s) in Itinerary_table.
function Itinerary_table_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Itinerary_table (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

if isempty(handles.ItineraryTableData)
    return;
end

if eventdata.Indices(1) > handles.stateNum
    return;
end

% change sequence
if eventdata.Indices(2)==2 && ~isempty(eventdata.NewData)
    handles.ItineraryTableData(eventdata.Indices(1),1)=round(eventdata.NewData);
end

% change ON duration
if eventdata.Indices(2)==3 && ~isempty(eventdata.NewData)
    handles.ItineraryTableData(eventdata.Indices(1),2)=round(eventdata.NewData);
end

% change OFF duration
if eventdata.Indices(2)==4 && ~isempty(eventdata.NewData)
    handles.ItineraryTableData(eventdata.Indices(1),3)=round(eventdata.NewData);
end

guidata(hObject, handles);
