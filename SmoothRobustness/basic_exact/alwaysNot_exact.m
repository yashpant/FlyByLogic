function robustness = alwaysNot_exact(t, states, P, I)
% alwaysNot_exact(t, states, P, I) 
% evaluates the exact robustness 'always not in P'

%inputs:
% t: time point where operator 'alwaysNot' is evaluated
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-steps) in which formula is to be evaluated

%outputs:
% robustness, the exact robustness value 

import casadi.*
if(isfloat(states))
    ap_robustness = zeros(length(I),1);
else
    ap_robustness = MX.zeros(length(I),1);
end
for i = 1:length(I)
    t_prime = t+I(i);
    ap_robustness(i) = inP_exact(states(t_prime, :), P);
end
% alwaysNot == min(-val)
robustness = min(-ap_robustness);
end