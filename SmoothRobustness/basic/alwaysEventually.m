function robustness_smooth = alwaysEventually(states, P, I, J, C)
% alwaysEventually(states, P, I, J, C)
% evaluates the smooth robustness

%inputs:
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-step increments) in which formula is to be evaluated
% C, constant for the smooth approxs of max/min

%outputs:
% robustness_smooth, the smooth robustness value 

ap_robustness = zeros(size(states, 1), 1);
lb = I(1)+J(1);
ub = I(end)+J(end);
for i = lb:ub 
    t = 1+i;
    ap_robustness(t) = inP(states(t, :), P, C);
end

r = zeros(length(I), 1);
for i = 1:length(I)
    r(i) = SmoothMax(ap_robustness(1+I(i)+J), C);
end
robustness_smooth = SmoothMin(r, C);
end