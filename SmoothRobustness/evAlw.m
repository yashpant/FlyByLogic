function [robustness, robustness_smooth] = evAlw(states, P, I, J, C)
for i = 1:length(I)
    % time point in which we check 'always'
    % I may start with 0, since it is time increments, steps in the future
    % Since MATLAB indices start with 1, we do 1+I(i)
    t = 1+I(i);
    [rob(i), rob_smooth(i)] = alw(t, states, P, J, C);
end
% ev = max
robustness = max(rob);
robustness_smooth = SmoothMax(rob_smooth, C);
end