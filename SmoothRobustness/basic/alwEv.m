function [robustness, robustness_smooth] = alwEv(states, P, I, J, C)
rob = zeros(length(I), 1);
rob_smooth = zeros(length(I), 1);
for i = 1:length(I)
    % i: time point in which we check 'eventually'
    % I may start with 0, since it is time increments, steps in the future
    % Since MATLAB indices start with 1, we do 1+I(i)
    t = 1+I(i);
    [rob(i), rob_smooth(i)] = ev(t, states, P, J, C);
end
% alw = min
robustness = min(rob);
robustness_smooth = SmoothMin(rob_smooth, C);
end