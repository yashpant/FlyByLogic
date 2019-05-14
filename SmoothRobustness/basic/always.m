function robustness_smooth = always(t, states, P, I, C)
% always(t, states, P, I, C) 
% evaluates the smooth robustness

%inputs:
% t: time index (not actual time) where operator 'always' is evaluated
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-step increments) in which formula is to be evaluated
% C, constant for the smooth approxs of max/min

%outputs:
% robustness_smooth, the smooth robustness value 

import casadi.*
if(isfloat(states))
    ap_robustness = zeros(length(I),1);
else
    ap_robustness = MX.zeros(length(I),1);
end
for i = 1:length(I)
    t_prime = t+I(i);
    ap_robustness(i) = inP(states(t_prime, :), P, C);
end
% always == min
robustness_smooth = SmoothMin(ap_robustness, C);
end