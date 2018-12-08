function varargout = Mission_Gui(varargin)
% MISSION_GUI MATLAB code for Mission_Gui.fig
%      MISSION_GUI, by itself, creates a new MISSION_GUI or raises the existing
%      singleton*.
%
%      H = MISSION_GUI returns the handle to a new MISSION_GUI or the handle to
%      the existing singleton*.
%
%      MISSION_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MISSION_GUI.M with the given input arguments.
%
%      MISSION_GUI('Property','Value',...) creates a new MISSION_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Mission_Gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Mission_Gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Mission_Gui

% Last Modified by GUIDE v2.5 07-Dec-2018 22:03:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Mission_Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Mission_Gui_OutputFcn, ...
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


% --- Executes just before Mission_Gui is made visible.
function Mission_Gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Mission_Gui (see VARARGIN)

addpath('../MiscFunctions');
addpath('../Maps_mrsl');
addpath('../CaseStudy/subfunctions');
addpath('../CaseStudy');
addpath('../Missions');

% Choose default command line output for Mission_Gui
handles.output = hObject;
     
% Populate Table with Defaults
Mission_Table  = findobj('Tag','latex_Mission_Table_tag');
d = {'Drone1','[2,2,6]','[2 4]','[0 5; 5 10]'};
set(Mission_Table, 'Data', d);
cur_cols = get(Mission_Table, 'ColumnName');
set(Mission_Table, 'ColumnName', cur_cols(1:4))

dat = Mission_Table.Data;
t_size = size(dat);

drone_goals = cell(t_size(1),1);

for i = 1:t_size(1)
    init_pos(:,i) = str2num(dat{i,2})'; %#ok<AGROW>
    V_bounds{i} = str2num(dat{i,3});
    for j = 1:t_size(2)-3%%%%%%%%%%%%%%%why this%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
        if ischar(dat{i,j+3})
            old_dat = str2num(dat{i,j+3});
        else
            old_dat = dat{i,j+3};
        end
        
        if (size(old_dat))
            new_dat = j*ones(size(old_dat) + [0 1]);
            new_dat(:,2:end) = old_dat;
            drone_goals{i} = [drone_goals{i}; new_dat];
        end
    end
end

handles.myhandle.V_bounds = V_bounds;
handles.myhandle.drone_goals = drone_goals;
handles.myhandle.init_pos = init_pos;
handles.myhandle.N_drones = 1;

Goal_Table  = findobj('Tag','Goal_Table_tag');
set(Goal_Table,'ColumnName', {'Goals'});
goal_table = {'[-1,-1,0,1,1,1]'};
set(Goal_Table, 'Data', goal_table);

Obstacle_Table  = findobj('Tag','Obstacle_Table_tag');
set(Obstacle_Table,'ColumnName', {'Obstacles'});
obstacle_table = {'[-1,-1,3,1,1,4]'};
set(Obstacle_Table, 'Data', obstacle_table);
handles.myhandle.ext_blocks = [-1,-1,3,1,1,4];

% default map
handles.myhandle.map_name = 'Empty_map.txt';

% default goal
goal{1}.stop = [0,0,1]';
goal{1}.lb = [-1,-1,0]';
goal{1}.ub = [1,1,1]';
goal{1}.col = 'green';
handles.myhandle.goal = goal;

dminEditText = findobj('Tag','dminEditText');
dminEditText.String = '0.1';
handles.myhandle.d_min = 0.1;

edit8 = findobj('Tag','edit8');
edit8.String = '1';
handles.myhandle.T = 1;

edit9 = findobj('Tag','edit9');
edit9.String = '30';
handles.myhandle.C = 30;

samplingEditText = findobj('Tag','samplingEditText');
samplingEditText.String = '0.05';
handles.myhandle.sampling_time = 0.05;

horizonEditText = findobj('Tag','horizonEditText');
horizonEditText.String = '20';
handles.myhandle.Horizon = 20;

handles = updateEnvironment(handles);
view(handles.disp_axes, -60, 10);
axis vis3d;

set(handles.missionStatus_data,'String', 'Not Ready'); %"" causes errors?
set(handles.missionnameEditText, 'String', 'default');
handles.myhandle.mission_name = 'default';
set(handles.missionLoaded_data, 'String', 'default');

handles.missionToLoad = '../Missions/default.mat';
handles %#ok<NOPRT>
% Update handles structure
%Code to add Latex in static text boxes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% handles.laxis = axes('parent',hObject,'units','normalized','position',[0 0 1 1],'visible','off');
% % Find all static text UICONTROLS whose 'Tag' starts with latex_
% lbls = findobj(hObject,'-regexp','tag','latex_*');
% for i=1:length(lbls)
%       l = lbls(i);
%       % Get current text, position and tag
%       set(l,'units','normalized');
%       s = get(l,'string');
%       p = get(l,'position');
%       t = get(l,'tag');
%       % Remove the UICONTROL
%       delete(l);
%       % Replace it with a TEXT object 
%       handles.(t) = text(p(1),p(2),s,'interpreter','latex');
% end
guidata(hObject, handles);

% UIWAIT makes Mission_Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Mission_Gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function handles = update_mission(handles)
%  collect drone goal interval data

% Get mission table object
Mission_Table  = findobj('Tag','latex_Mission_Table_tag');

% Get data from mission table
dat = Mission_Table.Data;

% Size of mission table
t_size = size(dat);

% Make a container to keep track of goal intervals for each drone
drone_goals = cell(t_size(1),1);

for i = 1:t_size(1)
    if (size(dat{i,2}))
        init_pos(:,i) = str2num(dat{i,2})';
    else
        disp('Please specify all initial positions')
    end
    
    for j = 1:t_size(2)-2
        if ischar(dat{i,j+2})
            old_dat = str2num(dat{i,j+2});
        else
            old_dat = dat{i,j+2};
        end
        
        if (size(old_dat))
            new_dat = j*ones(size(old_dat) + [0 1]);
            new_dat(:,2:end) = old_dat;
            drone_goals{i} = [drone_goals{i}; new_dat];
        end
    end
end

disp('Updated Mission Table');
handles.myhandle.init_pos = init_pos;
handles.myhandle.drone_goals = drone_goals;

% Resize the table for numDrones and num Goals
function handles = update_mission_table(handles)

% Get numDrones and numGoals
N_drones = get(handles.ndronesPopupmenu, 'Value');
N_goals = get(handles.ngoalsPopupmenu, 'Value');

% handles.myhandle.N_drones = N_drones;

% Get mission table object
Mission_Table  = findobj('Tag','latex_Mission_Table_tag');

% Get data from mission table
cur_table = get(Mission_Table, 'Data');
cur_cols = get(Mission_Table, 'ColumnName');

% Get size of current table
[N1,M1] = size(cur_table);

% Make new table
new_table = cell(N_drones, N_goals+2);
[N2,M2] = size(new_table);

for i = 1:N2
    if(i <= N1)
        new_table(i,1:min(M1,M2)) = cur_table(i,1:min(M1,M2));
    else
        new_table(i,1) = {['Drone',num2str(i)]};
    end
end

cur_cols(3:end) = [];

% Update Mission table header
for i = 1: N_goals
    cur_cols(i+2) = {['Goal',num2str(i)]};
end


set(Mission_Table, 'ColumnName', cur_cols);
set(Mission_Table, 'Data', new_table);



% --- Executes on selection change in ndronesPopupmenu.
function ndronesPopupmenu_Callback(hObject, eventdata, handles) %#ok<*DEFNU>
% hObject    handle to ndronesPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ndronesPopupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ndronesPopupmenu

handles.myhandle.N_drones = get(hObject, 'Value');
handles = update_mission_table(handles);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function ndronesPopupmenu_CreateFcn(hObject, eventdata, handles) %#ok<*INUSD>
% hObject    handle to ndronesPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});


% --- Executes on selection change in ngoalsPopupmenu.
function ngoalsPopupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to ngoalsPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ngoalsPopupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ngoalsPopupmenu

handles = update_numGoals(handles);
handles = update_mission_table(handles);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function ngoalsPopupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ngoalsPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});


% --- Executes on selection change in mapPopupmenu.
function mapPopupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to mapPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mapPopupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mapPopupmenu

map_cell = get(handles.mapPopupmenu, 'String');
handles.myhandle.map_name = map_cell{get(handles.mapPopupmenu, 'Value')};

handles = updateEnvironment(handles);

guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function mapPopupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mapPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

filename = dir('../Maps_mrsl/*.txt');
names = {filename.name};

set(hObject, 'String', names);


% --- Executes when entered data in editable cell(s) in latex_Mission_Table_tag.
function latex_Mission_Table_tag_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to latex_Mission_Table_tag (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

handles = update_mission(handles);

guidata(hObject, handles)

function handles = update_numGoals(handles)

% Get the number of goals from gui
N_goals = get(handles.ngoalsPopupmenu, 'Value');

% Update number of goals for mission
handles.myhandle.N_goals = N_goals;

% Get Goal Table Object
Goal_Table  = findobj('Tag','Goal_Table_tag');

% Get Data from Table Objact
cur_table = get(Goal_Table, 'Data');

% get size of current table
t_size = size(cur_table);

% Make new table of desired size
new_table = cell(N_goals, t_size(2));

% Populate the New Table with old table values (as many as can fit)
for i = 1:min(N_goals,t_size(1))
    new_table(i,:) = cur_table(i,:);
end

% Update the Goal Table with the new table
set(Goal_Table, 'Data', new_table);

% Populate handles with updated goals
function handles = update_goals(handles)

% Get Goal Table Object
Goal_Table  = findobj('Tag','Goal_Table_tag');

% Get Data from Table Object
dat = Goal_Table.Data;

% Get the size of the table
t_size = size(dat);

% Make structure to store goals
goal = cell(t_size(1),1);

% Get all the goals
for i = 1:t_size(1)
    if ischar(dat{i,1})
        array = (str2num(dat{i,1}))'; %#ok<*ST2NM>
    else
        array = (dat{i,1})';
    end
    goal{i}.lb = array(1:3);
    goal{i}.ub = array(4:6);
    goal{i}.stop = 0.5*(array(1:3) + array(4:6));
    goal{i}.col = 'green';
end

disp('Updated goals');
handles.myhandle.goal = goal;

handles = updateEnvironment(handles);

% --- Executes when entered data in editable cell(s) in Goal_Table_tag.
function Goal_Table_tag_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Goal_Table_tag (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

handles = update_goals(handles);

guidata(hObject, handles)

function handles = updateEnvironment(handles)
% Plot the environment on the figure

% Bring up desired axes
axes(handles.disp_axes); hold on;
cla;

map_name = handles.myhandle.map_name;
goal = handles.myhandle.goal;
ext_blocks = handles.myhandle.ext_blocks;

[AZ, EL] = view;
[map, obs] = plot_env(goal, map_name, ext_blocks);
view(AZ,EL);

% if obs is empty (hide an )

handles.myhandle.map = map;
handles.myhandle.obs = obs;


function horizonEditText_Callback(hObject, eventdata, handles)
% hObject    handle to horizonEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of horizonEditText as text
%        str2double(get(hObject,'String')) returns contents of horizonEditText as a double

handles.myhandle.Horizon = str2num(hObject.String);
disp('Updated Mission Horizon');
guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function horizonEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to horizonEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dminEditText_Callback(hObject, eventdata, handles)
% hObject    handle to dminEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dminEditText as text
%        str2double(get(hObject,'String')) returns contents of dminEditText as a double

handles.myhandle.d_min = str2num(hObject.String);
disp('Updated Drone Minimum Separation');
guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function dminEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dminEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function samplingEditText_Callback(hObject, eventdata, handles)
% hObject    handle to samplingEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of samplingEditText as text
%        str2double(get(hObject,'String')) returns contents of samplingEditText as a double

handles.myhandle.sampling_time = str2num(hObject.String);
disp('Updated Sampling time');
guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function samplingEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to samplingEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when entered data in editable cell(s) in Obstacle_Table_tag.
function Obstacle_Table_tag_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Obstacle_Table_tag (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

handles = update_obstacle_table(handles);

guidata(hObject, handles)

function handles = update_obstacle_table(handles)

N_ext_obs = get(handles.nobsPopupmenu, 'Value');
handles.myhandle.N_ext_obs = N_ext_obs;

Obstacle_Table  = findobj('Tag','Obstacle_Table_tag');
cur_table = get(Obstacle_Table, 'Data');

% get size of current table
t_size = size(cur_table);

% make new table
new_table = cell(N_ext_obs, t_size(2));

for i = 1:min(N_ext_obs,t_size(1))
    new_table(i,:) = cur_table(i,:);
end

ext_blocks = zeros(N_ext_obs,6);

for i = 1:N_ext_obs
    if (size(new_table{i,1}))
        ext_blocks(i,:) = str2num(new_table{i,1})';
    end
end

set(Obstacle_Table, 'Data', new_table);

disp('Updated External Obstacles');
handles.myhandle.ext_blocks = ext_blocks;
handles = updateEnvironment(handles);

% --- Executes on selection change in nobsPopupmenu.
function nobsPopupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to nobsPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns nobsPopupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from nobsPopupmenu

handles = update_obstacle_table(handles);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function nobsPopupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nobsPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});


% --- Executes on button press in cleanButton.
function cleanButton_Callback(hObject, eventdata, handles)
% hObject    handle to cleanButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Bring up desired axes
axes(handles.disp_axes);
cla;
disp('Map Cleaned');

handles = updateEnvironment(handles);

guidata(hObject, handles)


% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)
% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Bring up desired axes
axes(handles.disp_axes);
cla;
disp('Map Cleaned');

handles = updateEnvironment(handles);

disp('Plotting Mission...');
handles.myhandle.rob = plotMission(handles);

guidata(hObject, handles)


% --- Executes on button press in planButton.
function planButton_Callback(hObject, eventdata, handles)
% hObject    handle to planButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


set(handles.missionStatus_data, 'String', 'Not Ready');

% Do routine error checks
% Check all 
disp('Planning Mission....');
[handles.myhandle.w_opt, handles.myhandle.optParams, handles.myhandle.time_taken] = planMission(handles.myhandle);
Plan_Time = handles.myhandle.time_taken
set(handles.missionStatus_data, 'String', 'Ready');

guidata(hObject, handles)



function missionnameEditText_Callback(hObject, eventdata, handles)
% hObject    handle to missionnameEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of missionnameEditText as text
%        str2double(get(hObject,'String')) returns contents of missionnameEditText as a double

handles.myhandle.mission_name = get(hObject, 'String');

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function missionnameEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to missionnameEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.myhandle.missionTable = get(handles.latex_Mission_Table_tag, 'Data');
handles.myhandle.obstacleTable = get(handles.Obstacle_Table_tag, 'Data');
handles.myhandle.goalTable = get(handles.Goal_Table_tag, 'Data');

myfile = strcat('../Missions/', handles.myhandle.mission_name);
matObj = matfile(myfile, 'Writable', true);
matObj.myhandle = handles.myhandle;

filename = dir('../Missions/*.mat');
names = {filename.name};

set(handles.loadPopupmenu,'String',names);


% --- Executes on button press in loadButton.
function loadButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

dd = load(handles.missionToLoad);
handles.myhandle = dd.myhandle;

% EDIT DISPLAY
% load map_name into pop_up
maps = get(handles.mapPopupmenu, 'String');
index = find(strcmp(maps,handles.myhandle.map_name));
set(handles.mapPopupmenu, 'value', index);

% load # of drones
set(handles.ndronesPopupmenu, 'value', handles.myhandle.N_drones);

% load # of goals
set(handles.ngoalsPopupmenu, 'value', numel(handles.myhandle.goal));

% load # of ext_obs
set(handles.nobsPopupmenu, 'value', numel(handles.myhandle.ext_blocks(:,1)));

% load Horizon
set(handles.horizonEditText, 'String', handles.myhandle.Horizon);

% load Samp. Freq
set(handles.samplingEditText, 'String', handles.myhandle.sampling_time);

% load drone min sep
set(handles.dminEditText, 'String', handles.myhandle.d_min);

% load mission name
set(handles.missionnameEditText, 'String', handles.myhandle.mission_name);

% set mission load print
set(handles.missionLoaded_data, 'String', handles.myhandle.mission_name);

% populate tables
set(handles.latex_Mission_Table_tag, 'Data', handles.myhandle.missionTable);
set(handles.Obstacle_Table_tag, 'Data', handles.myhandle.obstacleTable);
set(handles.Goal_Table_tag, 'Data', handles.myhandle.goalTable);

% check if mission is ready
try
    handles.myhandle.w_opt;
    set(handles.missionStatus_data, 'String', 'Ready');
catch
    set(handles.missionStatus_data, 'String', 'Not Ready');
end

% Update environment
handles = updateEnvironment(handles);

guidata(hObject, handles)

% --- Executes on button press in uploadButton.
function uploadButton_Callback(hObject, eventdata, handles)
% hObject    handle to uploadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.missionStatus_data,'String', 'Not Ready');
set(handles.missionLoaded_data, 'String', handles.myhandle.mission_name);

% --- Executes on selection change in loadPopupmenu.
function loadPopupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to loadPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns loadPopupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from loadPopupmenu

contents = cellstr(get(hObject,'String'));
handles.missionToLoad = ['../Missions/', contents{get(hObject,'Value')}];

guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function loadPopupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to loadPopupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

filename = dir('../Missions/*.mat');
names = {filename.name};

set(hObject,'String',names);



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double
handles.myhandle.T = str2num(hObject.String);
disp('Updated Mission Horizon');
guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
