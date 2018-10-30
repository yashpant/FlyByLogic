%% start case study multiquads with eth tracker++, boolean


clc;close all;clear all;
N_drones = 2;
import casadi.*
addpath('subfunctions');
addpath('../MiscFunctions');
addpath('../Maps_mrsl');
%%
disp('Initializing...');
map_name = 'Maps_mrsl/map1.txt';
H_formula = 20; %H seconds %works with 5 for d=2, but use small C and then recompute. 6 is ok
obs = getObstacles(map_name);
map = load_map(map_name, .5, .5, 0);
close all;
plot_path(map, []);
close all;

goal{1}.stop = [4;4;2]; %delivery goal
goal{1}.ds = 0.5; %thickeness
goal{1}.lb = goal{1}.stop-goal{1}.ds;
goal{1}.ub = goal{1}.stop+goal{1}.ds;

goal{2}.stop = [0;0;4]; %surveliance one
goal{2}.ds = 0.5; %thickeness
goal{2}.lb = goal{2}.stop-goal{2}.ds;
goal{2}.ub = goal{2}.stop+goal{2}.ds;

goal{3}.stop = [4;-4;2]; %surveliance two
goal{3}.ds = 0.5; %thickeness
goal{3}.lb = goal{3}.stop-goal{3}.ds;
goal{3}.ub = goal{3}.stop+goal{3}.ds;

goal{4}.stop = [-4;4;1]; %delivery base
goal{4}.ds = 0.5; %thickeness
goal{4}.lb = goal{4}.stop-goal{4}.ds;
goal{4}.ub = goal{4}.stop+goal{4}.ds;


col{1} = 'blue';col{2} = 'yellow';
col{3} = 'yellow'; col{4} = 'green';
if(1)
for i = 1:4
goal{i}.poly = Polyhedron('lb',goal{i}.lb,'ub',goal{i}.ub);
hold on;
plot(goal{i}.poly,'Color',col{i},'alpha',0.5);
end
hold on;
for i = 1:size(obs,1)
plot(obs{i}.shape,'Color','red','alpha',0.5);
hold on;
end
data = map.boundary;
marg = 0.25;
axis ([data(1)-marg data(4)+marg data(2)-marg data(5)+marg data(3)-marg data(6)+marg]);
clear data;
end
h = 1/20; %dt



%% traj gen constraints in casadi
disp('Formulating in Casadi...');

%initialize tracker variables
T = 1; %1s duration of motion
M1 = (1/2*T^5)*[90 0 -15*T^2;-90*T 0 15*T^3;30*T^2 0 -3*T^4];

%tracker limits
max_per_axis = 1;
max_vel = 1; 
optParams.max_vel = max_vel;
max_accl = 2;
optParams.max_accl = max_accl;
% from vel constraints on pf
K1_T = (90/48)*(1/T) - (90/12)*(1/T) +(30/4)*(1/T);
optParams.K1_T = K1_T;
% from accl constraints on pf
aa = (90/4)*(1/T^5);
bb = -(90/2)*(1/T^4);
cc = (30/2)*(1/T^3);
tp1 = (-bb+sqrt(bb^2-4*aa*cc))/(2*aa);
tp2 = (-bb-sqrt(bb^2-4*aa*cc))/(2*aa);
t_prime = tp1*(tp1>=0)*(tp1<=T) + tp2*(tp2>=0)*(tp2<=T); %pick the right one
K2_tprime = (90/12)*(t_prime^3)/(T^5) - (90/4)*(t_prime^2)/(T^4) + ...
    (30/2)*(t_prime^1)/(T^3);
optParams.K2_tprime = K2_tprime;
%p0 = [-1.5;0;1]; %init position
p0 = zeros(3,N_drones);
p0(:,1) = [-4;-4;3]; %init position
p0(:,2) = [-4;4;2];

p0(:,3) = [1.25;0;4];
p0(:,4) = [-1.25;0;4];

p0(:,5) = [1.25;0;1.75];
p0(:,6) = [0;-1.25;1.75];

p0(:,7) = [2.25;2;1.5];
p0(:,8) = [2;-2.25;1.75];

p0(:,9) = [1;1;1.75];
p0(:,10)= [0;0;1.75];
p0(:,11) = [-1.25;-1.0;1.75];
p0(:,12) = [1;-1;1.75];
if(0) %rand init of quads
   p0 = random_init_generator(map,obs,N_drones); 
    
end

%init vels (zero)
v0 = zeros(3,N_drones);


%(1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;...
    %60*T^2 -24*T^3 3*T^4];
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
optParams.max_vel = max_vel;
optParams.max_per_axis = max_per_axis;

for i = 1:numel(obs)
optParams.obs_lb_N{i} = repmat(obs{i}.lb,Nsteps+1,1);
optParams.obs_ub_N{i} = repmat(obs{i}.ub,Nsteps+1,1);
end
for i = 1:numel(goal)
optParams.goal{i}.goal_lb_N = repmat(goal{i}.lb',Nsteps+1,1);
optParams.goal{i}.goal_ub_N = repmat(goal{i}.ub',Nsteps+1,1);
end

optParams.N_drones = N_drones;
optParams.d_min = 0.1; %min mutual separation
Clen = 3*(H_formula+1);
optParams.Clen = Clen;
%%  casadi shit
p_0 = MX.sym('p_0',3,1);
v_0 = MX.sym('v_0',3,1);

w = [];
v = [];
lbw = [];
ubw = [];
lbv = [];
ubv = [];
g = [];
lbg = [];
ubg = [];

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

%temp constraints to ensure end point, removed for the real stuff
% lbw(end-3+1:end) = goal.stop;
% ubw(end-3+1:end) = goal.stop;

var = [w;v];
var_ub = [ubw;ubv];
var_lb = [lbw;lbv];

%bool stuff
g = [g;-case_robustness(var,optParams)];
lbg = [lbg;eps];
ubg = [ubg;inf];
%% Casadi stuff
opts.ipopt.print_level = 5;
opts.print_time = false;
opts.expand = false;
options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
    'print_level',0)); %mumps, limited-memory
options.print_time = false;
prob = struct('f', 0, 'x', var, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

%% get init sol
disp('Getting init solution...');
w0 = [];
vv0 = [];
for i = 1:N_drones
    mission_type = 1*rem(i,2) + 2*(rem(i,2)==0);
    [temp tempv] = case_get_init_waypoints([p0(:,i);v0(:,i)],optParams,mission_type);
    if(sum(isnan(temp))>0 || sum(isnan(tempv))>0)
    disp('init infeasible');
    keyboard;
    end
    w0 = [w0;temp];
    vv0 = [vv0;tempv];
    pos0{i} = reshape(temp,3,H_formula+1);
    vel0{i} = reshape(tempv,3,H_formula+1);
   
%     if(i>=3)
% hold all;
%     plot3(pos0{i}(1,:),pos0{i}(2,:),pos0{i}(3,:),'-.');end
end

var0 = [w0;vv0];
 case_robustness(var0,optParams)

%% solve the nlp
disp('Solving...');
tic;
sol = solver('x0',var0,'lbx', var_lb, 'ubx', var_ub,...
    'lbg', lbg, 'ubg', ubg);

time_taken = toc
w_opt = full(sol.x);
%%
disp('Plotting...');
if(1)
    pause;
   
    waypoints = cell(optParams.N_drones,1);
    mar{1} = 'k*';
    mar{2} = 'g*';
    mar{3} = 'r*';
    mar{4} = 'b*';
    mar{5} = 'c*';
    mar{6} = 'm*';
    mar{7} = 'y*';
    mar{8} = 'r*';
    mar{9} = 'g*';
    mar{10} = 'k*';
    mar{11} = 'b*';
    mar{12} = 'c*';
    for d = 1:optParams.N_drones
        
        waypoints{d} = reshape(w_opt(1+(d-1)*(H_formula+1)*3:...
            (d)*(H_formula+1)*3),3,H_formula+1);
        hold all;
        %plot3(waypoints{d}(1,:),waypoints{d}(2,:),waypoints{d}(3,:),mar{d});
        
        
    end
    [negative_rob,xx,yy,zz] = case_robustness(w_opt,optParams);
    
    pause(0.1);
    mar{1} = 'ko';
    mar{2} = 'g*';
    mar{3} = 'r+';
    mar{4} = 'bd';
    mar{5} = 'cs';
    mar{6} = 'mp';
    mar{7} = 'yh';
    mar{8} = 'rp';
    mar{9} = 'g^';
    mar{10}= 'md';
    mar{11} = 'b*';
    mar{12} = 'y*';
    for d = 1:optParams.N_drones
        
        hold on;plot3(xx(:,d),yy(:,d),zz(:,d),'-.','linewidth',0.25);
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
%data = map.boundary;
%axis([data(1)-0.1 data(4)+0.1 data(2)-0.1 data(5)+0.1 data(3)-0.1 data(6)+0.1]);
    
end
 
