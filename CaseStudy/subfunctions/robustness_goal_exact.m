function [ rho_goal ] = robustness_goal_exact(xx,yy,zz,d,g,I,optParams)
% with respect to goal g
% 
import casadi.*

type_of = optParams.type_of;
if(type_of)
        temp = zeros(numel(I),1);
        temp_unsafe = zeros(size(optParams.obs,1),1);
    else
        temp = MX.sym('temp',numel(I),1);
        temp_unsafe = MX.sym('temp_unsafe',size(optParams.obs,1),1);
end


C1 = optParams.C1;
C = optParams.C;

% eventually goal in x y z
    rho_lb_xx = xx(I,d)-optParams.goal{g}.goal_lb_N(I,1);
    rho_ub_xx = optParams.goal{g}.goal_ub_N(I,1)-xx(I,d);
    rho_lb_yy = yy(I,d)-optParams.goal{g}.goal_lb_N(I,2);
    rho_ub_yy = optParams.goal{g}.goal_ub_N(I,2)-yy(I,d);
    rho_lb_zz = zz(I,d)-optParams.goal{g}.goal_lb_N(I,3);
    rho_ub_zz = optParams.goal{g}.goal_ub_N(I,3)-zz(I,d);
    
    
    % make this more efficient
    for i = 1:numel(rho_lb_xx)
        temp_vec = [rho_lb_xx(i) rho_ub_xx(i) rho_lb_yy(i) rho_ub_yy(i) ...
            rho_lb_zz(i) rho_ub_zz(i)];
        temp(i) = min(temp_vec);
        
    end
    rho_goal = max(temp);




end

