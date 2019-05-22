function ap_robustness = inP_exact(state, P)
% robustness of the AP 'given state is inside the Polyhedron P'
ps = P.b - P.A*state(:);
% ap_robustness: exact version
ap_robustness = min(ps);
end