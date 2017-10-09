function [ rho_unsafe ] =robustness_unsafe(xx,yy,zz,d,optParams)
%%
import casadi.*
type_of = optParams.type_of;
if(type_of)
        temp = zeros(size(xx,1),size(optParams.obs,1));
        temp_unsafe = zeros(size(optParams.obs,1),1);
    else
        temp = MX.sym('temp',size(xx,1),size(optParams.obs,1));
        temp_unsafe = MX.sym('temp_unsafe',size(optParams.obs,1),1);
end

C = optParams.C;
for j = 1:size(optParams.obs,1)

%   Detailed explanation goes here

  % always not unsafe in x y z
    rho_lb_xx = xx(:,d)-optParams.obs_lb_N{j}(:,1);
    rho_ub_xx = optParams.obs_ub_N{j}(:,1)-xx(:,d);
    %rho_xx = SmoothMin([rho_lb_xx;rho_ub_xx],C);
    
    rho_lb_yy = yy(:,d)-optParams.obs_lb_N{j}(:,2);
    rho_ub_yy = optParams.obs_ub_N{j}(:,2)-yy(:,d);
    %rho_yy = SmoothMin([rho_lb_yy;rho_ub_yy],C);
    
    rho_lb_zz = zz(:,d)-optParams.obs_lb_N{j}(:,3);
    rho_ub_zz = optParams.obs_ub_N{j}(:,3)-zz(:,d);
    %rho_zz = SmoothMin([rho_lb_zz;rho_ub_zz],C);
    
    
    
    % make this more efficient
    for i = 1:numel(rho_lb_xx)
        temp_vec = [rho_lb_xx(i) rho_ub_xx(i) rho_lb_yy(i) rho_ub_yy(i) ...
            rho_lb_zz(i) rho_ub_zz(i)];
        temp(i,j) = SmoothMin(temp_vec,C);        
    end
    temp_unsafe(j) = SmoothMin(-temp(:,j),C);
end
rho_unsafe = SmoothMin(temp_unsafe,C);  

