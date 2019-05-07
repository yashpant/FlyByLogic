clc;close all;clear all;

% be in this files folder
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
%% test some basic smooth robustness
states = [2*ones(50,2);0*ones(50,2); -2*ones(50, 2)]; 
P = Polyhedron('lb',[-1 -1],'ub',[1 1]); %Polyhedron that makes an AP

plot(P);
hold on;
plot(states(:,1),states(:,2),'b*');
axis([-2 2 -2 2]);
legend('Set','States')

%% Ex 1
I = 0:50;
J = 0:50;
fprintf('eventually[%i,%i] always[%i,%i] BE_IN_SET\n', I(1), I(end), J(1), J(end));
C = 100; % smooth constant

[robustness, robustness_smooth] = evAlw(states, P, I, J, C);
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 

%% Ex 2
I = 0:50;
J = 0:49;
fprintf('eventually[%i,%i] always[%i,%i] BE_IN_SET\n', I(1), I(end), J(1), J(end));
C = 100; % smooth constant

[robustness, robustness_smooth] = evAlw(states, P, I, J, C);
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 


