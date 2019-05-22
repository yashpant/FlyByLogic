function robustness = eventually_exact(t, states, P, I)
% eventually_exact(t, states, P, I) 
% evaluates the exact robustness

%inputs:
% t: time index (not actual time) where operator 'eventually' is evaluated
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-step increments) in which formula is to be evaluated

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
% eventually == max
robustness = max(ap_robustness);
end

