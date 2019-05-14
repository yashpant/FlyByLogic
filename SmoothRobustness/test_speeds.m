%% test_speeds()
% compares efficient implementation with by-the-def implementation
%%
function test_speeds()
clc;close all; clear all;
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
% test some basic smooth robustness
N = 100000;
M = 3;
% define a random signal
states = randi(10, N, M);

PP = sort(randi(10, 2, M), 1);
P = Polyhedron('lb', PP(1, :),'ub', PP(2, :)); %Polyhedron that makes an AP
C = 100; % smooth constant

% define a random interval that is meaningful (lb<ub)
JJ = sort(randi(100, 2, 1), 'ascend');
J = JJ(1):JJ(2);
LI = randi(10, 1, 1);
UI = N - J(end)-1;
I = LI:UI;

% TESTS
disp('EVENTUALLY ALWAYS')
disp('Implementation by definition...');
tic
robustness_smooth = eventuallyAlways_def(states, P, I, J, C);
toc
fprintf('smooth robustness: %.5f \n', robustness_smooth); 
disp('Efficient implementation...');
tic
robustness_smooth = eventuallyAlways(states, P, I, J, C);
toc
fprintf('smooth robustness: %.5f \n\n', robustness_smooth); 
%-------------------------------------
disp('ALWAYS EVENTUALLY')
disp('Implementation by definition...');
tic
robustness_smooth = alwaysEventually_def(states, P, I, J, C);
toc
fprintf('smooth robustness: %.5f \n', robustness_smooth); 
disp('Efficient implementation...');
tic
robustness_smooth = alwaysEventually(states, P, I, J, C);
toc
fprintf('smooth robustness: %.5f \n', robustness_smooth); 
end
%%




%% by-the-def implementation of 'alwaysEventually'
function robustness_smooth = alwaysEventually_def(states, P, I, J, C)

import casadi.*
if(isfloat(states))
    r = zeros(length(I),1);
else
    r = MX.zeros(length(I),1);
end
for i = 1:length(I)
    % i: time index for which we check 'eventually'
    % I may start with 0, since it is time increments, steps in the future
    % Since MATLAB indices start with 1, we do 1+I(i)
    t = 1+I(i);
    r(i) = eventually(t, states, P, J, C);
end
% always = min
robustness_smooth = SmoothMin(r, C);
end

%% by-the-def implementation of 'eventuallyAlways'
function robustness_smooth = eventuallyAlways_def(states, P, I, J, C)

import casadi.*
if(isfloat(states))
    r = zeros(length(I),1);
else
    r = MX.zeros(length(I),1);
end
for i = 1:length(I)
    % i: time index for which we check 'always'
    % I may start with 0, since it is time increments, steps in the future
    % Since MATLAB indices start with 1, we do 1+I(i)
    t = 1+I(i);
    r(i) = always(t, states, P, J, C);
end
% eventually = max
robustness_smooth = SmoothMax(r, C);
end