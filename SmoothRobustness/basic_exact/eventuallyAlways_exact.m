function robustness = eventuallyAlways_exact(states, P, I, J)
% eventuallyAlways_exact(states, P, I, J)
% evaluates the exact robustness

%inputs:
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-step increments) in which formula is to be evaluated

%outputs:
% robustness, the exact robustness value 

import casadi.*
if(isfloat(states))
    ap_robustness = zeros(size(states, 1), 1);
    r = zeros(length(I), 1);
else
    ap_robustness = MX.zeros(size(states, 1), 1);
    r = MX.zeros(length(I), 1);
end
%% find atomic proposition (ap) robustness 
lb = I(1)+J(1);
ub = I(end)+J(end);
for i = lb:ub 
    t = 1+i;
    ap_robustness(t) = inP_exact(states(t, :), P);
end
%% find formula robustness
for i = 1:length(I) 
    r(i) = min(ap(1+I(i)+J));
end
robustness = max(r);
end