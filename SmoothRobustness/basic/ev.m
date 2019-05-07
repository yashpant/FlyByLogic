function [rob, rob_smooth] = ev(t, states, P, I, C)
% evaluates the robustness: rho eventually_I (IN_SET, t)
% t: time point where operator 'eventually' is evaluated
r = zeros(length(I), 1);
r_smooth = zeros(length(I), 1);
for i = 1:length(I)
    t_prime = t+I(i);
    [r(i), r_smooth(i)] = inP(states(t_prime, :), P, C);
end
% eventually == max
rob = max(r);
rob_smooth = SmoothMax(r_smooth, C);
end