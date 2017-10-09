%% start reach avoid example for a single quad with eth tracker++

clc;close all;
optParams.N_drones = 1;
import casadi.*
addpath('../MiscFunctions');
addpath('../Maps_mrsl');
%%
disp('Initializing...');
map_name = 'Maps_mrsl/map0.txt';
H_formula = 6; %H_formula seconds
obs = getObstacles(map_name);
map = load_map(map_name, .5, .5, 0);
close all;
plot_path(map, []);
close all;
goal.stop = [1.75;1.75;0.75]; %map end point
goal.ds = .25; %thickeness
goal.lb = goal.stop-goal.ds;
goal.ub = goal.stop+goal.ds;
goal.poly = Polyhedron('lb',goal.lb,'ub',goal.ub);
hold on;
plot(goal.poly,'Color','green','alpha',0.5);
hold on;
plot(obs{1}.shape,'Color','red','alpha',0.5);
data = map.boundary;
axis ([data(1) data(4) data(2) data(5) data(3) data(6)]);
clear data;
h = 1/20; %1/h steps per T



%% traj gen constraints in casadi
disp('Formulating in Casadi...');
%initialize tracker variables
max_per_axis = .65;
%p0 = [-1.5;0;1]; %init position
p0 = [-1.25;-1.25;1.75]; %init position
T = 1.4; %1s duration of motion
M1 = (1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;...
    60*T^2 -24*T^3 3*T^4];
da = 0;
dv = 0; %start and stop from/at rest

Nsteps = H_formula*(T/h);
Nsteps = round(Nsteps)
% Populate optParams structure
optParams.T = T;
optParams.M1 = M1;
optParams.da = da;
optParams.dv = dv;
optParams.H_formula = H_formula;
optParams.N_per_T = T/h;%1/(T*h); %T/h; %(T/T)/H
optParams.goal = goal;
optParams.obs = obs;
optParams.map = map;
optParams.max_per_axis = max_per_axis;
% temp stuff
optParams.obs_lb_N = repmat(obs{1}.lb,Nsteps+1,1);
optParams.obs_ub_N = repmat(obs{1}.ub,Nsteps+1,1);
optParams.goal.goal_lb_N = repmat(goal.lb',Nsteps+1,1);
optParams.goal.goal_ub_N = repmat(goal.ub',Nsteps+1,1);


p_0 = MX.sym('p_0',3,1);
w = p_0;
lbw = p0;
ubw = p0;
g = [];
lbg = [];
ubg = [];
for k = 1:H_formula
    p_k = MX.sym(['p_' num2str(k)],3,1);
    w = [w;p_k];
    
    % dp for all axes
    dp_x = w(k*3+1) - w((k-1)*3+1);
    dp_y = w(k*3+2) - w((k-1)*3+2);
    dp_z = w(k*3+3) - w((k-1)*3+3);
    
    % Distance constraints per axis
    g = [g;[dp_x;dp_y;dp_z]];
    lbg = [lbg;-max_per_axis*ones(3,1)];
    ubg = [ubg;+max_per_axis*ones(3,1)];
    
    % overall bounds on movement
    lbw = [lbw;map.boundary(1:3)'];
    ubw = [ubw;map.boundary(4:6)'];
end

%temp constraints to ensure end point, removed for the real stuff
% lbw(end-3+1:end) = goal.stop;
% ubw(end-3+1:end) = goal.stop;

% Casadi stuff
opts.ipopt.print_level = 0;
opts.print_time = false;
opts.expand = false;
options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4,...
    'max_iter', 400, 'linear_solver', 'ma97', 'print_level', 0)); %ma97 fastest
options.print_time = false;
options.expand = false;
prob = struct('f', cost_reach_avoid(w,optParams), 'x', w, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

%% 
disp('Getting init solution...');
w0 = get_init_waypoints(p0,optParams);
%w0 = [p0;zeros(15,1)];
%% Solve the NLP
disp('Solving...');
tic;
sol = solver('x0',w0,'lbx', lbw, 'ubx', ubw,...
    'lbg', lbg, 'ubg', ubg);

time_taken = toc
w_opt = full(sol.x);
%%
disp('Plotting...');
if(1)
    pause;
    
    
    for i = 0:H_formula
        hold on;
        plot3(w_opt(i*3+1),w_opt(i*3+2),w_opt(i*3+3),'k*');
    end
    
    [negative_rob,xx,yy,zz] = cost_reach_avoid(w_opt,optParams);
    hold on;plot3(xx,yy,zz,'b');
    pause(0.1);
    for t = 1:numel(xx)
        pause(h);
        if(exist('gc','var'))
            delete(gc)
        end
        hold on;
        gc = plot3(xx(t),yy(t),zz(t),'ko');
    end
    
end





