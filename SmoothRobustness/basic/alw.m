function [rob, rob_smooth] = alw(t, states, P, I, C)
% evaluates the robustness: rho always_I (IN_SET, t)
% t: time point where operator 'always' is evaluated
r = zeros(length(I), 1);
r_smooth = zeros(length(I), 1);
for i = 1:length(I)
    t_prime = t+I(i);
    [r(i), r_smooth(i)] = inP(states(t_prime, :), P, C);
end
% always == min
rob = min(r);
rob_smooth = SmoothMin(r_smooth, C);
end