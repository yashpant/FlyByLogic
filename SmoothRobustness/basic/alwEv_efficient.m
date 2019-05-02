function [robustness, robustness_smooth] = alwEv_efficient(states, P, I, J, C)
AP = zeros(size(states, 1), 1);
lb = I(1)+J(1);
ub = I(end)+J(end);
for i = lb:ub 
    t = 1+i;
    AP(t) = inP(states(t, :), P, C);
end

rob = zeros(length(I), 1);
rob_smooth = zeros(length(I), 1);
for i = 1:length(I)
    rob(i) = max(AP(1+I(i)+J));
    rob_smooth(i) = SmoothMax(AP(1+I(i)+J), C);
end

robustness = min(rob);
robustness_smooth = SmoothMin(rob_smooth, C);
end