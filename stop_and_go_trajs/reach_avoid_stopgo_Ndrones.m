%% start reach avoid example for a multiquads with eth tracker++


clc;close all;clear all;
N_drones = 2;
import casadi.*
addpath('../MiscFunctions');
addpath('../Maps_mrsl');
%%
disp('Initializing...');
map_name = 'Maps_mrsl/map0.txt';
H_formula = 6; %H seconds %works with 5 for d=2, but use small C and then recompute. 6 is ok
obs = getObstacles(map_name);
map = load_map(map_name, .5, .5, 0);
close all;
plot_path(map, []);
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
h = 1/20; %dt


%% traj gen constraints in casadi
disp('Formulating in Casadi...');
%initialize tracker variables
max_per_axis = 1;
%p0 = [-1.5;0;1]; %init position
p0 = zeros(3,N_drones);
p0(:,1) = [-1.25;-1.25;1.75]; %init position
p0(:,2) = [1.25;-1.25;1.75];
p0(:,3) = [1.25;1.25;1.75];
p0(:,4) = [0;1.25;1.75];

T = 1; %1s duration of motion
M1 = (1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;...
    60*T^2 -24*T^3 3*T^4];
da = 0;
dv = 0; %start and stop from/at rest

Nsteps = H_formula*(T/h);

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
optParams.obs_lb_N = repmat(obs{1}.lb,Nsteps+1,1);
optParams.obs_ub_N = repmat(obs{1}.ub,Nsteps+1,1);
optParams.goal.goal_lb_N = repmat(goal.lb',Nsteps+1,1);
optParams.goal.goal_ub_N = repmat(goal.ub',Nsteps+1,1);
optParams.N_drones = N_drones;
optParams.d_min = 0.1; %min mutual separation
p_0 = MX.sym('p_0',3,1);
w = [];
lbw = [];
ubw = [];
g = [];
lbg = [];
ubg = [];
Clen = 3*(H_formula+1);
optParams.Clen = Clen;
for d = 1:N_drones
    lbw = [lbw;p0(:,d)];
    ubw = [ubw;p0(:,d)];
    p_0 = MX.sym(['p_' num2str(d) '_' num2str(0)],3,1);
    w = [w;p_0];
    for k = 1:H_formula
        p_k = MX.sym(['p_' num2str(d) '_' num2str(k)],3,1);
        w = [w;p_k];
        
        % dp for all axes
        dp_x = w(k*3+1+(d-1)*Clen) - w((k-1)*3+1+(d-1)*Clen);
        dp_y = w(k*3+2+(d-1)*Clen) - w((k-1)*3+2+(d-1)*Clen);
        dp_z = w(k*3+3+(d-1)*Clen) - w((k-1)*3+3+(d-1)*Clen);
        
        % Distance constraints per axis
        g = [g;[dp_x;dp_y;dp_z]];
        lbg = [lbg;-max_per_axis*ones(3,1)];
        ubg = [ubg;+max_per_axis*ones(3,1)];
        
        % overall bounds on movement
        lbw = [lbw;map.boundary(1:3)'];
        ubw = [ubw;map.boundary(4:6)'];
    end
end

%temp constraints to ensure end point, removed for the real stuff
% lbw(end-3+1:end) = goal.stop;
% ubw(end-3+1:end) = goal.stop;

% Casadi stuff
opts.ipopt.print_level = 5;
opts.print_time = false;
opts.expand = false;

opts.ipopt.print_level = 5;
opts.print_time = false;
opts.expand = false;
options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 2000, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
    'print_level',0)); %mumps, limited-memory
options.print_time = false;

prob = struct('f', cost_reach_avoid_Ndrones(w,optParams), 'x', w, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

%% Get init guess via lp
disp('Getting init solution...');
w0 = [];
for i = 1:N_drones
    temp = get_init_waypoints(p0(:,i),optParams);
    w0 = [w0;temp];
    pos0{i} = reshape(temp,3,H_formula+1);
    %hold all;
    %plot3(pos0{i}(1,:),pos0{i}(2,:),pos0{i}(3,:),'-.');
end





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
   
    waypoints = cell(optParams.N_drones,1);
    mar{1} = 'kd';
    mar{2} = 'gd';
    mar{3} = 'rd';
    mar{4} = 'bd';
    for d = 1:N_drones
        
        waypoints{d} = reshape(w_opt(1+(d-1)*(H_formula+1)*3:...
            (d)*(H_formula+1)*3),3,H_formula+1);
        hold all;
        plot3(waypoints{d}(1,:),waypoints{d}(2,:),waypoints{d}(3,:),mar{d});
        %plot3(w_opt(i*3+1),w_opt(i*3+2),w_opt(i*3+3),'*');
        
    end
    [negative_rob,xx,yy,zz] = cost_reach_avoid_Ndrones(w_opt,optParams);
    
    pause(0.1);
    mar{1} = 'ko';
    mar{2} = 'g*';
    mar{3} = 'r+';
    mar{4} = 'bd';
    for d = 1:optParams.N_drones
        
        hold on;plot3(xx(:,d),yy(:,d),zz(:,d));
        hold all;
    end
    for t = 1:size(xx,1)
        if(exist('gc','var'))
                        delete(gc)
        end
        for d = 1:optParams.N_drones
                               
            hold on;
            %gc(d) = 
            gc(d) = plot3(xx(t,d),yy(t,d),zz(t,d),mar{d},'MarkerSize',10);
            
        end
        pause(h); 
    end
    
end
 