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

% Last Modified by GUIDE v2.5 28-Mar-2018 05:12:07

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

% Choose default command line output for Mission_Gui
handles.output = hObject;
     
% Populate Table with Defaults
Mission_Table  = findobj('Tag','Mission_Table_tag');
d = {'Drone1','[0,0,0]','[0 5; 5 10]'};
set(Mission_Table, 'Data', d);
cur_cols = get(Mission_Table, 'ColumnName');
set(Mission_Table, 'ColumnName', cur_cols(1:3))

dat = Mission_Table.Data;
t_size = size(dat);

drone_goals = cell(t_size(1),1);

for i = 1:t_size(1)
    init_pos(:,i) = str2num(dat{i,2})';
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

handles.drone_goals = drone_goals;
handles.init_pos = init_pos;
handles.N_drones = 1;

Goal_Table  = findobj('Tag','Goal_Table_tag')
set(Goal_Table,'ColumnName', {'Goals'});
goal_table = {'[1,0,1]'};
set(Goal_Table, 'Data', goal_table);

Obstacle_Table  = findobj('Tag','Obstacle_Table_tag');
set(Obstacle_Table,'ColumnName', {'Obstacles'});
obstacle_table = {'[-1,-1,3,1,1,4]'};
set(Obstacle_Table, 'Data', obstacle_table);
handles.ext_blocks = [-1,-1,3,1,1,4];

% default map
handles.map_name = 'Empty_map.txt';

% default goal
goal{1}.stop = [0,0,1]';
goal{1}.ds = 0.5; %thickeness
goal{1}.lb = goal{1}.stop-goal{1}.ds;
goal{1}.ub = goal{1}.stop+goal{1}.ds;
goal{1}.col = 'green';
handles.goal = goal;

d_min_edit = findobj('Tag','d_min_edit_tag');
d_min_edit.String = '0.1';
handles.d_min = 0.1;

t_step_edit = findobj('Tag','t_step_edit_tag');
t_step_edit.String = '1';
handles.t_step = 1;

horizon_edit = findobj('Tag','horizon_edit_tag');
horizon_edit.String = '25';
handles.Horizon = 25;

sampling_time_edit = findobj('Tag','sampling_time_edit_tag');
sampling_time_edit.String = '0.05';
handles.sampling_time = 0.05;

handles = updateEnvironment(handles);
view(handles.disp_axes, -60, 10);
axis vis3d;

handles
% Update handles structure
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


% --- Executes on selection change in Ndrones_pop_up_tag.
function Ndrones_pop_up_tag_Callback(hObject, eventdata, handles)
% hObject    handle to Ndrones_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Ndrones_pop_up_tag contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Ndrones_pop_up_tag

N_drones = get(handles.Ndrones_pop_up_tag, 'Value');
handles.N_drones = N_drones;

Mission_Table  = findobj('Tag','Mission_Table_tag');
cur_table = get(Mission_Table, 'Data');

% get size of current table
t_size = size(cur_table);

% make new table
new_table = cell(N_drones, t_size(2));

for i = 1:N_drones
    if(i <= t_size(1))
        new_table(i,:) = cur_table(i,:);
    else
        new_table(i,1) = {['Drone',num2str(i)]};
    end
end

set(Mission_Table, 'Data', new_table);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function Ndrones_pop_up_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ndrones_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});


% --- Executes on selection change in Ngoals_pop_up_tag.
function Ngoals_pop_up_tag_Callback(hObject, eventdata, handles)
% hObject    handle to Ngoals_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Ngoals_pop_up_tag contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Ngoals_pop_up_tag

N_goals = get(handles.Ngoals_pop_up_tag, 'Value');
handles.N_goals = N_goals;

Goal_Table  = findobj('Tag','Goal_Table_tag');
cur_table = get(Goal_Table, 'Data');

% get size of current table
t_size = size(cur_table);

% make new table
new_table = cell(N_goals, t_size(2));

for i = 1:min(N_goals,t_size(1))
    new_table(i,:) = cur_table(i,:);
end

set(Goal_Table, 'Data', new_table);

% Update Mission Table
Mission_Table  = findobj('Tag','Mission_Table_tag');
cur_table = get(Mission_Table, 'Data');
cur_cols = get(Mission_Table, 'ColumnName');

% get size of current table
t_size = size(cur_table);

% make new table
new_table = cell(t_size(1), N_goals+2);
new_table(:,1:min(t_size(2),N_goals+2)) = cur_table(:,1:min(t_size(2),N_goals+2));

% Update Mission table header
for i = 1: N_goals
    cur_cols(i+2) = {['Goal',num2str(i)]};
end

set(Mission_Table, 'Data', new_table);
set(Mission_Table, 'ColumnName', cur_cols);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function Ngoals_pop_up_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ngoals_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});


% --- Executes on selection change in map_select_pop_up.
function map_select_pop_up_Callback(hObject, eventdata, handles)
% hObject    handle to map_select_pop_up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns map_select_pop_up contents as cell array
%        contents{get(hObject,'Value')} returns selected item from map_select_pop_up

map_cell = get(handles.map_select_pop_up, 'String');
handles.map_name = map_cell{get(handles.map_select_pop_up, 'Value')};

handles = updateEnvironment(handles);

guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function map_select_pop_up_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map_select_pop_up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'Empty_map.txt','map0.txt', 'map1.txt', 'map2.txt'});


% --- Executes when entered data in editable cell(s) in Mission_Table_tag.
function Mission_Table_tag_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Mission_Table_tag (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

%  collect drone goal interval data
Mission_Table  = findobj('Tag','Mission_Table_tag');

dat = Mission_Table.Data;
t_size = size(dat);

drone_goals = cell(t_size(1),1);

for i = 1:t_size(1)
    if (size(dat{i,2}))
        init_pos(:,i) = str2num(dat{i,2})';
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

handles.init_pos = init_pos;
handles.drone_goals = drone_goals;

guidata(hObject, handles)

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

%  collect drone goal interval data
Goal_Table  = findobj('Tag','Goal_Table_tag');

dat = Goal_Table.Data;
t_size = size(dat);

goal = cell(t_size(1),1);

for i = 1:t_size(1)
    if ischar(dat{i,1})
        goal{i}.stop = (str2num(dat{i,1}))'; %goal
    else
        goal{i}.stop = (dat{i,1})';
    end
    goal{i}.ds = 0.5; %thickeness
    goal{i}.lb = goal{i}.stop-goal{i}.ds;
    goal{i}.ub = goal{i}.stop+goal{i}.ds;
    goal{i}.col = 'green';
end

handles.goal = goal;

handles = updateEnvironment(handles);

guidata(hObject, handles)

function handles = updateEnvironment(handles)
% Plot the environment on the figure

% Bring up desired axes
axes(handles.disp_axes); hold on;
cla;

map_name = handles.map_name;
goal = handles.goal;
ext_blocks = handles.ext_blocks;

[AZ, EL] = view;
[map, obs] = plot_env(goal, map_name, ext_blocks);
view(AZ,EL);

% if obs is empty (hide an )

handles.map = map;
handles.obs = obs;


% --- Executes on button press in plan_pushbutton_tag.
function plan_pushbutton_tag_Callback(hObject, eventdata, handles)
% hObject    handle to plan_pushbutton_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles
[handles.w_opt, optParams, handles.time_taken] = planMission(handles);
Plan_Time = handles.time_taken

% Bring up desired axes
axes(handles.disp_axes);
handles.rob = plotMission(handles.w_opt, optParams);

guidata(hObject, handles)

function horizon_edit_tag_Callback(hObject, eventdata, handles)
% hObject    handle to horizon_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of horizon_edit_tag as text
%        str2double(get(hObject,'String')) returns contents of horizon_edit_tag as a double

handles.Horizon = str2num(hObject.String);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function horizon_edit_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to horizon_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d_min_edit_tag_Callback(hObject, eventdata, handles)
% hObject    handle to d_min_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d_min_edit_tag as text
%        str2double(get(hObject,'String')) returns contents of d_min_edit_tag as a double

handles.d_min_tag = str2num(hObject.String);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function d_min_edit_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d_min_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sampling_time_edit_tag_Callback(hObject, eventdata, handles)
% hObject    handle to sampling_time_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sampling_time_edit_tag as text
%        str2double(get(hObject,'String')) returns contents of sampling_time_edit_tag as a double

handles.sampling_time = str2num(hObject.String);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function sampling_time_edit_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sampling_time_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t_step_edit_tag_Callback(hObject, eventdata, handles)
% hObject    handle to t_step_edit_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t_step_edit_tag as text
%        str2double(get(hObject,'String')) returns contents of t_step_edit_tag as a double

handles.t_step = str2num(hObject.String)

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function t_step_edit_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t_step_edit_tag (see GCBO)
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

%  collect external obstacle data
Obstacle_Table  = findobj('Tag','Obstacle_Table_tag');

dat = Obstacle_Table.Data;
t_size = size(dat);

ext_blocks = zeros(t_size(1),6);

for i = 1:t_size(1)
    if (size(dat{i,1}))
        ext_blocks(i,:) = str2num(dat{i,1})';
    end
end

handles.ext_blocks = ext_blocks;
updateEnvironment(handles);
guidata(hObject, handles)

% --- Executes on selection change in ext_obs_pop_up_tag.
function ext_obs_pop_up_tag_Callback(hObject, eventdata, handles)
% hObject    handle to ext_obs_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ext_obs_pop_up_tag contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ext_obs_pop_up_tag

N_ext_obs = get(handles.ext_obs_pop_up_tag, 'Value');
handles.N_ext_obs = N_ext_obs;

Obstacle_Table  = findobj('Tag','Obstacle_Table_tag');
cur_table = get(Obstacle_Table, 'Data');

% get size of current table
t_size = size(cur_table);

% make new table
new_table = cell(N_ext_obs, t_size(2));

for i = 1:min(N_ext_obs,t_size(1))
    new_table(i,:) = cur_table(i,:);
end

set(Obstacle_Table, 'Data', new_table);

guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function ext_obs_pop_up_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ext_obs_pop_up_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'1', '2', '3', '4', '5', '6', '7', '8'});
