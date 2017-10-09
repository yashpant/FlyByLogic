function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
vx = zeros(4,6*map.nblocks); vy = zeros(4,6*map.nblocks); vz = zeros(4,6*map.nblocks);
%     faceMat = [1 2 3 1 1 5; 2 3 6 8 2 8; 3 6 5 4 7 6; 4 7 4 5 3 7];
 
C = zeros(4, 3);
C(1,:) = [1 0 0];
C(2,:) = [0 0 1];
C(3,:) = [1 0 1];
C(4,:) = [1 1 0];
for i = 1:map.nblocks
    lwrLimit = map.blocks(i,1:3);
    uprLimit = map.blocks(i,4:6);
        v    = [lwrLimit(1:3); ...
        uprLimit(1) lwrLimit(2) lwrLimit(3);...
        uprLimit(1) lwrLimit(2) uprLimit(3);...
        lwrLimit(1) lwrLimit(2) uprLimit(3);...
        lwrLimit(1) uprLimit(2) uprLimit(3);...
        uprLimit(1) uprLimit(2) uprLimit(3);...
        uprLimit(1) uprLimit(2) lwrLimit(3);...
        lwrLimit(1) uprLimit(2) lwrLimit(3)];
 
    startx = (i-4)*4+1;
    starty = (i-1)*6+1;
    vx(:,starty:starty+5) = [v(1,1) v(2,1) v(3,1) v(1,1) v(1,1) v(5,1); ...
                            v(2,1) v(3,1) v(6,1) v(8,1) v(2,1) v(8,1);...
                            v(3,1) v(6,1) v(5,1) v(5,1) v(7,1) v(7,1);...
                            v(4,1) v(7,1) v(4,1) v(4,1) v(8,1) v(6,1)];
    vy(:,starty:starty+5) = [v(1,2) v(2,2) v(3,2) v(1,2) v(1,2) v(5,2); ...
                            v(2,2) v(3,2) v(6,2) v(8,2) v(2,2) v(8,2);...
                            v(3,2) v(6,2) v(5,2) v(5,2) v(7,2) v(7,2);...
                            v(4,2) v(7,2) v(4,2) v(4,2) v(8,2) v(6,2)];
    vz(:,starty:starty+5) = [v(1,3) v(2,3) v(3,3) v(1,3) v(1,3) v(5,3); ...
                            v(2,3) v(3,3) v(6,3) v(8,3) v(2,3) v(8,3);...
                            v(3,3) v(6,3) v(5,3) v(5,3) v(7,3) v(7,3);...
                            v(4,3) v(7,3) v(4,3) v(4,3) v(8,3) v(6,3)];
end
figure();
hold on;
grid on; grid minor;
xlabel('x');
ylabel('y');
zlabel('z');
hold on;
fill3(vx,vy,vz,[1 1 1]);
alpha(.24);
for i = 1:length(path)
    if (size(path{i}))
        plot3(path{i}(:,1), path{i}(:,2),path{i}(:,3), '-', 'linewidth', 3); 
    end
end
data = map.boundary;
axis ([data(1) data(4) data(2) data(5) data(3) data(6)]);
%axis equal
end

% % Alternative
% function plot_path(map, path)
% % PLOT_PATH Visualize a path through an environment
% %   PLOT_PATH(map, path) creates a figure showing a path through the
% %   environment.  path is an N-by-3 matrix where each row corresponds to the
% %   (x, y, z) coordinates of one point along the path.
% 
% x_max = map.xlen;
% y_max = map.ylen;
% z_max = map.zlen;
% 
% dim = x_max*y_max*z_max;
% 
% X = nan(dim, 1);
% Y = nan(dim, 1);
% Z = nan(dim, 1);
% C = nan(dim, 3);
% 
% m_grid = map.m_grid;
% boundary = map.boundary;
% p = 1;
% 
% for i = 1:x_max
%     for j = 1:y_max
%         for k = 1:z_max
%             
%             mycolor = m_grid(i, j, k, 4);
%             if ~isnan(mycolor)
%                 X(p) = m_grid(i, j, k, 1);
%                 Y(p) = m_grid(i, j, k, 2);
%                 Z(p) = m_grid(i, j, k, 3);
%                 R = floor(mycolor/1000000);
%                 G = floor((mycolor - R*1000000)/1000);
%                 B = mycolor - R*1000000 - G*1000;
%                 C(p,:) = [R G B]/255.0;
%                 p = p + 1;
%             end
%             
%         end
%     end
% end
%           
% X(p:end) = [];
% Y(p:end) = [];
% Z(p:end) = [];
% C(p:end,:) = [];
% 
% figure
% grid on; grid minor;
% xlabel('x');
% ylabel('y');
% zlabel('z');
% scatter3(X, Y, Z, 10, C, 'P');
% hold on
% if (size(path))
%     plot3(path(:,1), path(:,2),path(:,3), '-k', 'linewidth', 3); 
% end
% axis([boundary(1) boundary(4) boundary(2) boundary(5) boundary(3) boundary(6)])
% xlabel('x');
% ylabel('y');
% zlabel('z');
% end

