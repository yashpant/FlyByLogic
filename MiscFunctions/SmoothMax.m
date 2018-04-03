function [x_min,C] = SmoothMax(vec_x,C) 
%x_max = SoftMin(vec_x,C)
% also decimates C>0 if C is large that matlab says log(sum(exp(C*vec_x))) is
% Inf

%%

x_min = (1/C)*log(sum(exp(C*vec_x)));
