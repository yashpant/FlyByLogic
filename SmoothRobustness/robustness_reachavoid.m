function [rho,rho_goal,rho_obstacle] = robustness_reachavoid(states,goal,obstacle,I_goal)
% spec is always_I (x NOT \in obstacle) AND eventually_(I_goal) (x \in goal) 


%% awlays not in obstacle

[rho_obstacle,~] = always_Inot(states,obstacle,1:length(states),100);
%% eventually in goal

[rho_goal,~] = eventually_I(states,goal,I_goal,1000);

%% overall robustness
rho = SmoothMin([rho_goal;rho_obstacle],1000);

