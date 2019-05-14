function ap_robustness = inP(state, P, C)
% robustness of the AP 'given state is inside the Polyhedron P'
ps = P.b - P.A*state(:);
% ap_robustness: smooth version
ap_robustness = SmoothMin(ps, C);
end