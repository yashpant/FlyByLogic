function [negative_rob,xx,yy,zz] = Mission_Robustness_exact(var,optParams)
%%
import casadi.*
type_of = isfloat(var); %0 for casadi
optParams.type_of = type_of;
H = optParams.H_formula; %better be even
w = var(1:numel(var)/2);
v = var(numel(var)/2+1:end);

% assign intervals for goals
N_per_T = optParams.N_per_T;

% 50 works for 2 drones
C = 2.0; %const for smooth min/max operation %for 2 drones, use 10 with a period of 5s, casadi is unstable numerically
C1 =2.0; %const for smooth max %20 works for 10 drones, det init points
C2 =2.0; %5 makes no numerical instblty in 12 drones
optParams.C = C;
optParams.C1 = C1;
optParams.C2 = C2;

Nobs = size(optParams.obs,1); %number of obstacles
dT = optParams.T/optParams.N_per_T:optParams.T/optParams.N_per_T:optParams.T;
dv = optParams.dv;
da = optParams.da;
M1 = optParams.M1;
T = optParams.T;
Clen = optParams.Clen;

if(type_of) %if double input
    temp_x = zeros(optParams.N_drones, numel(dT));
    temp_y = zeros(optParams.N_drones, numel(dT));
    temp_z = zeros(optParams.N_drones, numel(dT));
    xx = zeros(numel(dT)+1,optParams.N_drones);
    yy = zeros(numel(dT)+1,optParams.N_drones);
    zz = zeros(numel(dT)+1,optParams.N_drones);
    rho_unsafe = zeros(optParams.N_drones,1);
    rho_goal = zeros(optParams.N_drones,1);
    mutual_distances = zeros(numel(dT)*optParams.H_formula+1,1);
    if (optParams.N_drones > 1)
        dists = zeros(nchoosek(optParams.N_drones,2),1);
    end
else
    temp_x = MX.sym('temp_x',optParams.N_drones, numel(dT));
    temp_y = MX.sym('temp_y',optParams.N_drones, numel(dT));
    temp_z = MX.sym('temp_z',optParams.N_drones, numel(dT));
    xx = MX.sym('xx',numel(dT)*optParams.H_formula+1,optParams.N_drones);
    yy = MX.sym('yy',numel(dT)*optParams.H_formula+1,optParams.N_drones);
    zz = MX.sym('zz',numel(dT)*optParams.H_formula+1,optParams.N_drones);
    rho_unsafe = MX.sym('r_u',optParams.N_drones,1);
    rho_goal = MX.sym('r_u',optParams.N_drones,1);
    mutual_distances = MX.sym('msep',numel(dT)*optParams.H_formula+1,1);
    if (optParams.N_drones > 1)
        dists = MX.sym('mdist',nchoosek(optParams.N_drones,2),1);
    end
end

% For each Drone
for d = 1:optParams.N_drones
    %init posns
    xx(1,d) = w(1+(d-1)*Clen);
    yy(1,d) = w(2+(d-1)*Clen);
    zz(1,d) = w(3+(d-1)*Clen);
    
    %get all sampled splines
    for k = 1:optParams.H_formula
        
        % dp for all axes
        dp_x = w(k*3+1+(d-1)*Clen) - w((k-1)*3+1+(d-1)*Clen) - T*v((k-1)*3+1+(d-1)*Clen);
        dp_y = w(k*3+2+(d-1)*Clen) - w((k-1)*3+2+(d-1)*Clen) - T*v((k-1)*3+2+(d-1)*Clen);
        dp_z = w(k*3+3+(d-1)*Clen) - w((k-1)*3+3+(d-1)*Clen) - T*v((k-1)*3+3+(d-1)*Clen);
        
        % constants for all 3 axes
        al_x = M1(1,:)*[dp_x;dv;da];
        be_x = M1(2,:)*[dp_x;dv;da];
        gam_x = M1(3,:)*[dp_x;dv;da];
        al_y = M1(1,:)*[dp_y;dv;da];
        be_y = M1(2,:)*[dp_y;dv;da];
        gam_y = M1(3,:)*[dp_y;dv;da];
        al_z = M1(1,:)*[dp_z;dv;da];
        be_z = M1(2,:)*[dp_z;dv;da];
        gam_z = M1(3,:)*[dp_z;dv;da];
        
        temp_x(d,:) = (al_x/120)*dT.^5 + (be_x/24)*dT.^4 + ...
            (gam_x/6)*dT.^3   + w((k-1)*3+1+(d-1)*Clen) + ...
            dT*v((k-1)*3+1+(d-1)*Clen); %fix w points
        
        xx(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_x(d,:)';
        
        temp_y(d,:) = (al_y/120)*dT.^5 + (be_y/24)*dT.^4 + ...
            (gam_y/6)*dT.^3   + w((k-1)*3+2+(d-1)*Clen) + ...
            dT*v((k-1)*3+2+(d-1)*Clen); % fix w points
        
        yy(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_y(d,:)';
        
        temp_z(d,:) = (al_z/120)*dT.^5 + (be_z/24)*dT.^4 + ...
            (gam_z/6)*dT.^3   + w((k-1)*3+3+(d-1)*Clen) + ...
            dT*v((k-1)*3+3+(d-1)*Clen); %fix w points
        
        zz(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_z(d,:)'; 
        
    end
    
    % Robusteness unsafe set
    rho_unsafe(d) = robustness_unsafe_exact(xx,yy,zz,d,optParams);
    
    
    % Robustness Get to goal in interval
    i = 1;
    rho = [];
    drone_goals = optParams.drone_goals{d};
    if size(drone_goals)
        for g = drone_goals(:,1)'
            I = 1+drone_goals(i,2)*N_per_T/T:1+drone_goals(i,3)*N_per_T/T;
            rho = [rho; robustness_goal_exact(xx,yy,zz,d,g,I,optParams)];
            i = i + 1;
        end
    end
    rho_goal(d) = min(rho);
    
end

% pairwise distances
if (optParams.N_drones > 1)
    combos = nchoosek(1:optParams.N_drones,2);
    for p = 1:size(combos,1)
        for k=1:size(xx,1) %for all time steps
            pa = [xx(k,combos(p,1));yy(k,combos(p,1));zz(k,combos(p,1))];
            pb = [xx(k,combos(p,2));yy(k,combos(p,2));zz(k,combos(p,2))];
            mutual_distances(k) = norm(pa-pb,2)-optParams.d_min;
        end
        dists(p) = min(mutual_distances);
    end
end

sep_rob = min(dists)
d_min = optParams.d_min

if (optParams.N_drones > 1)
    negative_rob = -min([rho_unsafe;rho_goal;dists]);
else
    negative_rob = -min([rho_unsafe;rho_goal]);
end