function [p0] = random_init_generator(map,obs,N_drones)
dx = 0.25;

% region
Region = Polyhedron('lb',map.boundary(1:3),'ub',map.boundary(4:6));

%free space
FreeSpace = Region;
for i = 1:size(obs)
    FreeSpace =  Region\obs{i}.shape;
    
end
if(0)
    plot(FreeSpace);
end
nRects = numel(FreeSpace);
ubs = zeros(nRects,3)';
lbs = zeros(nRects,3)';
for j = 1:nRects
    lbs(:,j) = FreeSpace(j).Internal.lb;
    ubs(:,j) = FreeSpace(j).Internal.ub;
    
end

ct = 0;
clear gx gy gz
for i = 1:nRects
    gx{i} = [lbs(1,i):0.25:ubs(1,i)];
    gy{i} = [lbs(2,i):0.25:ubs(2,i)];
    gz{i} = [lbs(3,i):0.25:ubs(3,i)];
    
    
end

p0 = zeros(3,N_drones);

for i = 1:N_drones
    
    i1(i) = randi(nRects);
    i2x(i) = randi(numel(gx{i1(i)})-1);
    i2y(i) = randi(numel(gy{i1(i)})-1);
    i2z(i) = randi(numel(gz{i1(i)})-1);
    
    
    if(i>1)
        while( (i1(i)==i1(i-1)) && (i2x(i-1)==i2x(i)) && ...
                (i2y(i-1)==i2y(i)) && (i2z(i-1)==i2z(i)))
            i1(i) = randi(nRects);
            i2x(i) = randi(numel(gx{i1(i)})-1);
            i2y(i) = randi(numel(gy{i1(i)})-1);
            i2z(i) = randi(numel(gz{i1(i)})-1);
            
        end
    end
    
    p0(1,i) = (gx{i1(i)}(i2x(i))+gx{i1(i)}(i2x(i)+1))/2;
    p0(2,i) = (gy{i1(i)}(i2y(i))+gy{i1(i)}(i2y(i)+1))/2;
    p0(3,i) = (gz{i1(i)}(i2z(i))+gz{i1(i)}(i2z(i)+1))/2;
    
end

