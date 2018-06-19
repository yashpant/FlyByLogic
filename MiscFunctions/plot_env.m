function [map, obs] = plot_env(goal, map_name, varargin)

ext_blocks = [];

if size(varargin)
    ext_blocks = varargin{1};
end

% Get Obstacles from the map
obs = getObstacles(map_name, ext_blocks);

% Load the Map
map = load_map(map_name, .5, .5, 0);

% Plot goals with color if goals exist
if(~isempty(goal{1}))
    for i = 1:length(goal)
        goal{i}.poly = Polyhedron('lb',goal{i}.lb,'ub',goal{i}.ub);
        plot(goal{i}.poly,'Color',goal{i}.col,'alpha',0.5); hold on;
    end
end

% Plot obstacles
for i = 1:size(obs,1)
    plot(obs{i}.shape,'Color','red','alpha',0.5);
end

% Set bounds with margin m
b = map.boundary;
m = 0.25;

axis ([b(1)-m b(4)+m b(2)-m b(5)+m b(3)-m b(6)+m]);




