function [w_opt, optParams, time_taken] = planMission(missionHandle)
import casadi.*

% Initialize Variables

% How many drones?
N_drones = missionHandle.N_drones;

% Drone Minimum Separation
d_min = missionHandle.d_min;

% How long is the mission?
H_formula = missionHandle.Horizon;

% Set Sampling Time 
h = missionHandle.sampling_time;

% Separation of waypoints
T = 5; %1s duration of motion

% Map and obstacles
map = missionHandle.map;
obs = missionHandle.obs;

% goals
goal = missionHandle.goal;

drone_goals = missionHandle.drone_goals;

% init_pos
init_pos = missionHandle.init_pos;
%% Generate Constraints for Trajectory

% Initialize Tracker Variables

M1 = (1/(2*T^5))*[90 0 -15*T^2;-90*T 0 15*T^3;30*T^2 0 -3*T^4];

% Tracker limits
max_per_axis = 1;
max_vel = 5; 
max_accl = 10;

% From vel constraints on pf
K1_T = (90/48)*(1/T) - (90/12)*(1/T) +(30/4)*(1/T)

% From accl constraints on pf
aa =  (90/4)*(1/T^5);
bb = -(90/2)*(1/T^4);
cc =  (30/2)*(1/T^3);

tp1 = (-bb+sqrt(bb^2-4*aa*cc))/(2*aa);
tp2 = (-bb-sqrt(bb^2-4*aa*cc))/(2*aa);

% Pick the right one
t_prime = tp1*(tp1>=0)*(tp1<=T) + tp2*(tp2>=0)*(tp2<=T); 

K2_tprime = (90/12)*(t_prime^3)/(T^5) - (90/4)*(t_prime^2)/(T^4) + ...
    (30/2)*(t_prime)/(T^3)

% Set Initial Random Initial Positions
if(0) 
    % Randon Initial Position of Drones
    p0 = random_p0_generator(map,obs,N_drones);
else
    p0 = init_pos;
end

% Initial Velocities (Start from Rest)
v0 = zeros(3,N_drones);

% Zero Initial Jerk and Acceleration
da = 0;
dv = 0;

% Total Number of steps (per Drone)
Nsteps = H_formula*(T/h);

Clen = 3*(H_formula+1);

% Populate optParams structure
optParams.N_drones = N_drones;
optParams.d_min = d_min;
optParams.Clen = Clen;
optParams.T = T;
optParams.M1 = M1;
optParams.K1_T = K1_T;
optParams.K2_tprime = K2_tprime;
optParams.da = da;
optParams.dv = dv;
optParams.H_formula = H_formula;
optParams.N_per_T = T/h;
optParams.goal = goal;
optParams.drone_goals = drone_goals;
optParams.obs = obs;
optParams.map = map;
optParams.max_vel = max_vel; 
optParams.max_accl = max_accl;
optParams.max_per_axis = max_per_axis;
optParams.sampling_time = h;

for i = 1:numel(obs)
optParams.obs_lb_N{i} = repmat(obs{i}.lb,Nsteps+1,1);
optParams.obs_ub_N{i} = repmat(obs{i}.ub,Nsteps+1,1);
end

for i = 1:numel(goal)
optParams.goal{i}.goal_lb_N = repmat(goal{i}.lb',Nsteps+1,1);
optParams.goal{i}.goal_ub_N = repmat(goal{i}.ub',Nsteps+1,1);
end

%%  Formulate Optimization Problem in Casadi 
disp('Formulating in Casadi...');

% Initial Position and Velocity Casadi Def
p_0 = MX.sym('p_0',3,1); %#ok<*NASGU>
v_0 = MX.sym('v_0',3,1);

%#ok<*AGROW>

% Initialize some Empty Containers
w = []; % Waypoint Container
v = []; % Velocity Container
lbw = []; % Lower Bound on Waypoints
ubw = []; % Upper Bound on Waypoints
lbv = []; % Lower Bound on Velocities
ubv = []; % Upper Bound on Velocities
g = []; % Goal Container
lbg = []; % Lower Bound on Goals
ubg = []; % Upper Bound on Goals 

% Replicate Constraints for all drones
for d = 1:N_drones
    lbw = [lbw;p0(:,d)];
    ubw = [ubw;p0(:,d)];
    lbv = [lbv;v0(:,d)];
    ubv = [ubv;v0(:,d)];
    
    p_0 = MX.sym(['p_' num2str(d) '_' num2str(0)],3,1);
    v_0 = MX.sym(['v_' num2str(d) '_' num2str(0)],3,1);
    
    w = [w;p_0];
    v = [v;v_0];
    
    for k = 1:H_formula
        p_k = MX.sym(['p_' num2str(d) '_' num2str(k)],3,1);
        v_k = MX.sym(['v_' num2str(d) '_' num2str(k)],3,1);
        
        w = [w;p_k];
        v = [v;v_k];
        % dp for all axes
        dp_x = w(k*3+1+(d-1)*Clen) - w((k-1)*3+1+(d-1)*Clen);
        dp_y = w(k*3+2+(d-1)*Clen) - w((k-1)*3+2+(d-1)*Clen);
        dp_z = w(k*3+3+(d-1)*Clen) - w((k-1)*3+3+(d-1)*Clen);
        
        px_k = w(k*3+1+(d-1)*Clen);px_km1 = w((k-1)*3+1+(d-1)*Clen);
        py_k = w(k*3+2+(d-1)*Clen);py_km1 = w((k-1)*3+2+(d-1)*Clen);
        pz_k = w(k*3+3+(d-1)*Clen);pz_km1 = w((k-1)*3+3+(d-1)*Clen);
        % dv for all axes
        
        dv_x = v(k*3+1+(d-1)*Clen) - v((k-1)*3+1+(d-1)*Clen);
        dv_y = v(k*3+2+(d-1)*Clen) - v((k-1)*3+2+(d-1)*Clen);
        dv_z = v(k*3+3+(d-1)*Clen) - v((k-1)*3+3+(d-1)*Clen);
        
        vx_k = v(k*3+1+(d-1)*Clen);vx_km1 = v((k-1)*3+1+(d-1)*Clen);
        vy_k = v(k*3+2+(d-1)*Clen);vy_km1 = v((k-1)*3+2+(d-1)*Clen);
        vz_k = v(k*3+3+(d-1)*Clen);vz_km1 = v((k-1)*3+3+(d-1)*Clen);
        
        % constants for all 3 axes
        al_x = M1(1,:)*[dp_x-T*vx_km1;0;da];
        be_x = M1(2,:)*[dp_x-T*vx_km1;0;da];
        gam_x = M1(3,:)*[dp_x-T*vx_km1;0;da];
        al_y = M1(1,:)*[dp_y-T*vy_km1;0;da];
        be_y = M1(2,:)*[dp_y-T*vy_km1;0;da];
        gam_y = M1(3,:)*[dp_y-T*vy_km1;0;da];
        al_z = M1(1,:)*[dp_z-T*vz_km1;0;da];
        be_z = M1(2,:)*[dp_z-T*vz_km1;0;da];
        gam_z = M1(3,:)*[dp_z-T*vz_km1;0;da];
        
        %v_fs
        vf_x = (al_x/24)*T^4 + (be_x/6)*T^3 + (gam_x/2)*T^2 + vx_km1;
        vf_y = (al_y/24)*T^4 + (be_y/6)*T^3 + (gam_y/2)*T^2 + vy_km1;
        vf_z = (al_z/24)*T^4 + (be_z/6)*T^3 + (gam_z/2)*T^2 + vz_km1;
        % Constraints per axis and velocity dynamics
        
        g = [g;... % append
             K1_T*dp_x+(1-T*K1_T)*vx_km1;... %from vel constraints on dp
             K1_T*dp_y+(1-T*K1_T)*vy_km1;... 
             K1_T*dp_z+(1-T*K1_T)*vz_km1;... 
             K2_tprime*dp_x-T*K2_tprime*vx_km1;... %from acc constraints
             K2_tprime*dp_y-T*K2_tprime*vy_km1;...
             K2_tprime*dp_z-T*K2_tprime*vz_km1;...
             [vx_k-vf_x;vy_k-vf_y;vz_k-vf_z]]; %from vel dynamics
        lbg = [lbg;-max_vel*ones(3,1);-max_accl*ones(3,1);zeros(3,1)];
        ubg = [ubg;+max_vel*ones(3,1);+max_accl*ones(3,1);zeros(3,1)];
           
        % overall bounds on movement
        lbw = [lbw;map.boundary(1:3)'];
        ubw = [ubw;map.boundary(4:6)'];
        lbv = [lbv;-ones(3,1)*max_vel];
        ubv = [ubv;+ones(3,1)*max_vel];
    end
end


% Stack Decision Variables in 1 long array of Decision Variables
var = [w;v];
var_ub = [ubw;ubv];
var_lb = [lbw;lbv];

%% Configure Settings for Optimization Solver
opts.ipopt.print_level = 5;
opts.print_time = false;
opts.expand = false;
options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4,...
                    'max_iter', 5, 'linear_solver', 'mumps',...
                    'hessian_approximation','limited-memory',...
                    'print_level',5));
                
options.print_time = false;
prob = struct('f', Mission_Robustness(var,optParams), 'x', var, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

%% Get Initial Trajectory
tic;
disp('Getting init solution...');

w0 = [];
vv0 = [];

pos0 = cell(N_drones, 1);
vel0 = cell(N_drones, 1);

%  For Each Drone
for i = 1:N_drones
    mission_type = 1*rem(i,2) + 2*(rem(i,2)==0);
    [temp, tempv] = Mission_Get_Initial_Waypoints([p0(:,i);v0(:,i)],...
        optParams,mission_type);
    
    if(sum(isnan(temp))>0 || sum(isnan(tempv))>0)
        disp('init infeasible');
        keyboard;
    end
    
    w0 = [w0;temp];
    vv0 = [vv0;tempv];
    pos0{i} = reshape(temp,3,H_formula+1);
    vel0{i} = reshape(tempv,3,H_formula+1);
end

var0 = [w0;vv0];
optParams.var0 = var0;
Init_waypoints_rob = Mission_Robustness(var0,optParams);
toc
%% solve the nlp
disp('Solving...');
tic;
sol = solver('x0',var0,'lbx', var_lb, 'ubx', var_ub,...
    'lbg', lbg, 'ubg', ubg);

time_taken = toc;
w_opt = full(sol.x);