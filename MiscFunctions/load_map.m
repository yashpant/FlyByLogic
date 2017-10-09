function mymap = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

% # An example environment
% # boundary xmin ymin zmin xmax ymax zmax
% # block xmin ymin zmin xmax ymax zmax r g b

% Define containers
fid = fopen(filename);

mymap.xy_res = xy_res;
mymap.z_res = z_res;
mymap.margin = margin;
mymap.m_grid = zeros(0, 0, 0, 0);

m = margin;
boundary = [];
blocks = zeros(100, 9);
l = 1;

while ~feof(fid) % loop over the following until the end of the file is reached.
    line = fgets(fid); % read in one line
    if strfind(line,'#')
        continue
    elseif strfind(line,'boundary')
        B = str2num(line(9:end));
        boundary = B;

        x = B(1):xy_res:(B(4));
        if ~(x(end) == B(4))
%             x = [x B(4)];
        end
        y = B(2):xy_res:(B(5));
        if ~(y(end) == B(5))
%             y = [y B(5)];
        end
        z = B(3):z_res:(B(6));
        if ~(z(end) == B(6))
%             z = [z B(6)];
        end
        
        mymap.xlen = length(x);
        mymap.ylen = length(y);
        mymap.zlen = length(z);
        mymap.dim = mymap.xlen * mymap.ylen * mymap.zlen;
        
        m_grid = nan(mymap.xlen, mymap.ylen, mymap.zlen, 4);
        
        for i = 1:mymap.xlen
            for j = 1:mymap.ylen
                for k = 1:mymap.zlen
                    m_grid(i, j, k, 1:3) = [x(i) y(j) z(k)];
                end
            end
        end
        
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
mymap.boundary = boundary;
mymap.blocks = blocks;
mymap.nblocks = l-1;
for i = 1:l-1
    % Add blocks to map
    B = boundary;
    b = blocks(i,:);
    cb = 1000000*b(7) + 1000*b(8) + b(9);

    x_start = 1 + floor(single((b(1) - B(1)-m)/xy_res));
    y_start = 1 + floor(single((b(2) - B(2)-m)/xy_res));
    z_start = 1 + floor(single((b(3) - B(3)-m)/z_res));
    
    x_end = 1 + floor(single((b(4) - B(1)+m)/xy_res));
    y_end = 1 + floor(single((b(5) - B(2)+m)/xy_res));
    z_end = 1 + floor(single((b(6) - B(3)+m)/z_res));
    
    for a = x_start:x_end
        for b = y_start:y_end
            for c = z_start:z_end
                if ((a > 0) && (b > 0) && (c > 0)...
                        && (a <= mymap.xlen) && (b <= mymap.ylen)...
                        && (c <= mymap.zlen))
                    m_grid(a, b, c, 4) = cb;
                end
            end
        end
    end
end

mymap.m_grid = m_grid;

end
