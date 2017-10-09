%% start reach avoid example for a single quad with eth tracker++, feasible robustness only

clc;close all;
optParams.N_drones = 1;
import casadi.*
addpath('~/Documents/Github/AATC_quadrotors/Meam620_ATC_working/MTL_Examples/SubFunctions/');
addpath('../../../Miscellaneous/General/');
addpath('../Maps_mrsl');
addpath('../MiscCode');
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
h = 1/20; %mpc
[sys_d,~] = quad_lin_dynamics(0.03,h); %get discrete quad dyn at 20Hz


%% traj gen constraints in casadi
disp('Formulating in Casadi...');
%initialize tracker variables
max_per_axis = 0.75;
%p0 = [-1.5;0;1]; %init position
p0 = [-1.25;-1.25;1.75]; %init position
T = 1.4; %1s duration of motion
M1 = (1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;...
    60*T^2 -24*T^3 3*T^4];
da = 0;
dv = 0; %start and stop from/at rest

Nsteps = H_formula*(T/h);
Nsteps = round(Nsteps);
% Populate optParams structure
optParams.T = T;
optParams.M1 = M1;
optParams.da = da;
optParams.dv = dv;
optParams.H_formula = H_formula;
optParams.N_per_T = T/h;
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
    'max_iter', 400, 'linear_solver', 'ma27', 'print_level', 0, ...
    'hessian_approximation','limited-memory')); %ma57
%ma57,27,97,77 are faster than mumps faster than ma86
options.print_time = false;
options.expand = false;
% add robustness positivenss
% g = [g;-cost_reach_avoid(w,optParams)];
% lbg = [lbg;0.01];
% ubg = [ubg;inf];
prob = struct('f', cost_reach_avoid(w,optParams), 'x', w, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

% Solve the NLP
disp('Getting init solution...');
w0 = get_init_waypoints(p0,optParams);
if(sum(isnan(w0))>0)
   disp('init error!');
   keyboard;
end
%w0 = [p0;zeros((15,1)];
%% ros init stfuf
if(1)
    rosinit;
    rate = rosrate(1/optParams.T); %1s
    rate.OverrunAction = 'drop';
    
    goal_pub = rospublisher('crazym04/trackers_manager/line_tracker_eth1/goal');
    msg_pub = rosmessage(goal_pub);
%     msg_pub.x = 0;
%     msg_pub.y = 0;
%     msg_pub.z = 1;
%     msg_pub.relative = false;
%     send(goal_pub,msg_pub);
    
    state_sub = rossubscriber('crazym04/odom','nav_msgs/Odometry');
    msg_sub = receive(state_sub);
end
%%
%get an initial optimal guess
%
disp('Getting init. state');
msg_sub = receive(state_sub);
state_now = [msg_sub.Pose.Pose.Position.X;...
    msg_sub.Pose.Pose.Position.Y;...
    msg_sub.Pose.Pose.Position.Z];
w0(1:3) = state_now;
lbw(1:3) = state_now;
ubw(1:3) = state_now;
%%
disp('Solving init...');
tic;
sol_init = solver('x0',w0,'lbx', lbw, 'ubx', ubw,...
    'lbg', lbg, 'ubg', ubg);
time_taken = toc;
w_0 = full(sol_init.x);
w_init = w_0;
reshape(w_init,3,H_formula+1)
%% solve online
disp('Online')
delta = 0.0;
load('data/ra_6s.mat');
clc
for i = 0:H_formula-1
    %get current state
    i
    tic
    %tic;
    msg_sub = receive(state_sub);
    state_now = [msg_sub.Pose.Pose.Position.X;...
        msg_sub.Pose.Pose.Position.Y;...
        msg_sub.Pose.Pose.Position.Z];
    %toc;
    %state_now = w_0(1+3*(i):3+3*(i))+0.05*randn(3,1);%if offline sim
    
    %update bounds on waypoints
    w_0(1+3*(i):3+3*(i)) = state_now;
    
    lbw(1+3*(i):3+3*(i)) = state_now;
    ubw(1+3*(i):3+3*(i)) = state_now;
    time_getting_state = toc
    %solve optimization
    %disp('Solving init...');
    tic;
    sol = solver('x0',w_0,'lbx', lbw, 'ubx', ubw,... 
        'lbg', lbg, 'ubg', ubg);
    %temp=full(sol.g);
    %temp(end)
    %time_taken = toc
    %reuse init solution for next time
    time_optim = toc
    
    tic;
    w_0 = full(sol.x);
    wp_list{i+1} = full(sol.x);
    %next point"
    %disp('next point');
    waypoint_commanded{i+1} = (w_0(1+3*(i+1):3+3*(i+1)));
    msg_pub.X = w_0(1+3*(i+1));
    msg_pub.Y = w_0(2+3*(i+1));
    msg_pub.Z = w_0(3+3*(i+1));
    %msg_pub.X = waypoints_ra6(1,i+2);
    %msg_pub.Y = waypoints_ra6(2,i+2);
    %msg_pub.Z = waypoints_ra6(3,i+2);
    
    msg_pub.Duration = rosduration(optParams.T+1*delta);
    msg_pub.Relative = false;
    send(goal_pub,msg_pub);
    time_publish=toc
    pause(optParams.T+delta);
    %waitfor(rate);
end
rosshutdown;
%% plot that crap
disp('Plotting...');

if(1)
    hold on;
    plot(goal.poly,'Color','green','alpha',0.5);
    hold on;
    plot(obs{1}.shape,'Color','red','alpha',0.5);
    data = map.boundary;
    axis ([data(1) data(4) data(2) data(5) data(3) data(6)]);
    pause;
    w_opt = full(sol.x);
    
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





