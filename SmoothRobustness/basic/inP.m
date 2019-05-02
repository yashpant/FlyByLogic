function [r, r_smooth] = inP(state, P, C)
% robustness of the AP 'given state is inside the Polyhedron P'
ps = P.b - P.A*state(:);
% r:        robustness
% r_smooth: smooth version
r = min(ps);
r_smooth = SmoothMin(ps, C);
end