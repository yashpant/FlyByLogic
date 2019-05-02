% test speeds
clc;close all;clear all;
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
%% test some basic smooth robustness
N = 100000;
M = 3;
states = randi(100000, N, M);

PP = sort(randi(1000, 2, M), 1);
P = Polyhedron('lb', PP(1, :),'ub', PP(2, :)); %Polyhedron that makes an AP
C = 100; % smooth constant

JJ = sort(randi(100, 2, 1), 'ascend');
J = JJ(1):JJ(2);
LI = randi(10, 1, 1);
UI = N - J(end)-1;
I = LI:UI;

%% TESTS
tic
[robustness, robustness_smooth] = evAlw(states, P, I, J, C);
toc
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 

tic
[robustness, robustness_smooth] = evAlw_efficient(states, P, I, J, C);
toc
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 