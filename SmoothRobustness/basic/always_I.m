function [robustness,ap_robustness] = always_I(states,P,I,C)

%always_I

%inputs:
% states, (N x Dim) input of N time-steps, Dim number of dimensions
% P, (MPT Polyhedron w/H-rep) defining the atomic proposition
% I, interval (of time-steps) in which formula is to be evaluated
% C, constant for the smooth approxs of max/min

%outputs:
% robustness, the robustness value 
% ap_robustness, robustness of state(i,:) \in P at each time step 1,2,...,N

ap_robustness = zeros(size(states,1),1);

for i = I
   
    ps = P.b - P.A*states(i,:)';
    ap_robustness(i) = SmoothMin(ps,C);
    
end

robustness = SmoothMin(ap_robustness(I),C);