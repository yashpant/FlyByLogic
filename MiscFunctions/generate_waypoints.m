function [ ] = generate_waypoints( variable, fileName, flag )
%MATLAB2OPENCV Save `variable` to yml/xml file 
% fileName: filename where the variable is stored
% flag: `a` for append, `w` for writing.
%   Detailed explanation goes here

[rows, cols] = size(variable);

% Beware of Matlab's linear indexing
% variable = variable';

% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w'; 
end

if ( ~exist(fileName,'file') || flag == 'w' )
    % New file or write mode specified 
    file = fopen( fileName, 'w');
else
    % Append mode
    file = fopen( fileName, 'a');
end

% Write variable header
fprintf( file, 'way_x: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(1,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_y: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(2,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_z: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(3,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_yaw: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', 0);
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_vx: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(4,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_vy: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(5,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

% Write variable header
fprintf( file, 'way_vz: [');

% Write variable data
for i=1:cols
    fprintf( file, '%.6f', variable(6,i));
    if (i == cols), break, end
    fprintf( file, ', ');
    if mod(i,7) == 0
        fprintf( file, '\n ');
    end
end
fprintf( file, ']\n');

fprintf( file, '\n');

fprintf( file, 'sc: 1\n');
fprintf( file, 'sampling_time: 1\n');
fprintf( file, 'receeding_horizon: false\n');
fprintf( file, 'receeding_horizon_rate: 10\n');

fclose(file);
end