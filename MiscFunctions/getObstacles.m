function obs = getObstacles(map, varargin)

ext_blocks_flag = 0;
ext_blocks = [];

if size(varargin)
    ext_blocks = varargin{1};
    ext_blocks_flag = 1;
end

% Define containers
fid = fopen(map);
blocks = zeros(100, 9);
l = 1;

while ~feof(fid) % loop over the following until the end of the file is reached.
    line = fgets(fid); % read in one line
    if strfind(line,'#')
        continue
    elseif strfind(line,'boundary')
        B = str2num(line(9:end));
        continue
    elseif strfind(line,'block')
        b = str2num(line(6:end));
        blocks(l,:) = b;
        l = l + 1;     
    else
        continue
    end
end

fclose(fid);
blocks(l:end,:) = [];
blocks = blocks(:,1:6);

[N, ~] = size(blocks(:,1));


% Define Obstacles as Polyhedrons
obs = cell(N+size(ext_blocks,1),1); % For plotting 

% For simple map types
A = [-eye(3);eye(3)];
   
for i = 1:N
   obs{i}.lb = blocks(i,1:3); obs{i}.ub =  blocks(i,4:6);
   obs{i}.A = A;
   obs{i}.b = [-obs{i}.lb obs{i}.ub]';
   obs{i}.shape = Polyhedron('lb', obs{i}.lb,'ub', obs{i}.ub);
%    plot(obs{i}.shape, 'Color','white','Alpha',.9);
end

if (ext_blocks_flag && size(ext_blocks,1))
    for i = N+1:size(ext_blocks(:,1))+N
        obs{i}.lb = ext_blocks(i-N,1:3); obs{i}.ub =  ext_blocks(i-N,4:6);
        obs{i}.A = A;
        obs{i}.b = [-obs{i}.lb obs{i}.ub]';
        obs{i}.shape = Polyhedron('lb', obs{i}.lb,'ub', obs{i}.ub);
    end
end

axis([B(1) B(4) B(2) B(5) B(3) B(6)]);
axis equal
end
