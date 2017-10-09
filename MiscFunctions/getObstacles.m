function obs = getObstacles(map)

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

figure(1);
hold on;

% Define Obstacles as Polyhedrons
obs = cell(N,1); % For plotting 

% For simple map types
A = [-eye(3);eye(3)];
   
for i = 1:N
   obs{i}.lb = blocks(i,1:3); obs{i}.ub =  blocks(i,4:6);
   obs{i}.A = A;
   obs{i}.b = [-obs{i}.lb obs{i}.ub]';
   obs{i}.shape = Polyhedron('lb', obs{i}.lb,'ub', obs{i}.ub);
   plot(obs{i}.shape, 'Color','white','Alpha',.9);
end

axis([B(1) B(4) B(2) B(5) B(3) B(6)]);
axis equal
end
