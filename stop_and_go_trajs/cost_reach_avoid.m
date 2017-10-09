function [negative_rob,xx,yy,zz] = cost_reach_avoid(w,optParams)
import casadi.*
type_of = isfloat(w); %0 for casadi

C = 50; %const for smooth min/max operation
dT = optParams.T/optParams.N_per_T:optParams.T/optParams.N_per_T:optParams.T;
dv = optParams.dv;
da = optParams.da;
M1 = optParams.M1;
xx = w(1); %init positions
yy = w(2);
zz = w(3);
for k = 1:optParams.H_formula

    % dp for all axes
    
    dp_x = w(k*3+1) - w((k-1)*3+1);
    dp_y = w(k*3+2) - w((k-1)*3+2);
    dp_z = w(k*3+3) - w((k-1)*3+3);
    
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
    
    temp_x = (al_x/120)*dT.^5 + (be_x/24)*dT.^4 + ...
        (gam_x/6)*dT.^3   + w((k-1)*3+1); 
    xx = [xx;temp_x'];
    temp_y = (al_y/120)*dT.^5 + (be_y/24)*dT.^4 + ...
        (gam_y/6)*dT.^3   + w((k-1)*3+2); 
    yy = [yy;temp_y'];
    temp_z = (al_z/120)*dT.^5 + (be_z/24)*dT.^4 + ...
        (gam_z/6)*dT.^3   + w((k-1)*3+3); 
    zz = [zz;temp_z'];
    
end

% always not unsafe in x y z
rho_lb_xx = xx-optParams.obs_lb_N(:,1);
rho_ub_xx = optParams.obs_ub_N(:,1)-xx;
%rho_xx = SmoothMin([rho_lb_xx;rho_ub_xx],C);

rho_lb_yy = yy-optParams.obs_lb_N(:,2);
rho_ub_yy = optParams.obs_ub_N(:,2)-yy;
%rho_yy = SmoothMin([rho_lb_yy;rho_ub_yy],C);

rho_lb_zz = zz-optParams.obs_lb_N(:,3);
rho_ub_zz = optParams.obs_ub_N(:,3)-zz;
%rho_zz = SmoothMin([rho_lb_zz;rho_ub_zz],C);

if(type_of)
temp = zeros(numel(rho_lb_xx),1);
else
temp = MX.sym('temp',numel(rho_lb_xx),1);
end

% make this more efficient
for i = 1:numel(rho_lb_xx)
    temp_vec = [rho_lb_xx(i) rho_ub_xx(i) rho_lb_yy(i) rho_ub_yy(i) ...
        rho_lb_zz(i) rho_ub_zz(i)];
    temp(i) = SmoothMin(temp_vec,C);
    
end
rho_unsafe = SmoothMin(-temp,C);

%rho_unsafe = SmoothMin([rho_xx;rho_yy;rho_zz],C);

% eventually goal in x y z
rho_lb_xx = xx-optParams.goal.goal_lb_N(:,1);
rho_ub_xx = optParams.goal.goal_ub_N(:,1)-xx;
rho_lb_yy = yy-optParams.goal.goal_lb_N(:,2);
rho_ub_yy = optParams.goal.goal_ub_N(:,2)-yy;
rho_lb_zz = zz-optParams.goal.goal_lb_N(:,3);
rho_ub_zz = optParams.goal.goal_ub_N(:,3)-zz;


% make this more efficient
for i = 1:numel(rho_lb_xx)
    temp_vec = [rho_lb_xx(i) rho_ub_xx(i) rho_lb_yy(i) rho_ub_yy(i) ...
        rho_lb_zz(i) rho_ub_zz(i)];
    temp(i) = SmoothMin(temp_vec,C);
    
end
rho_goal = SmoothMax(temp,C);

negative_rob = -SmoothMin([rho_unsafe;rho_goal],C);
