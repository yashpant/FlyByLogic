clc;close all;clear all;

% be in this files folder
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
%% test some basic smooth robustness
states = [2*ones(5,2); % outsideP
          0*ones(5,2); % inP
          2*ones(5,2); % outsideP
          0*ones(5,2); % inP
          2*ones(5,2); % outsideP
          0*ones(5,2)]; %inP
P = Polyhedron('lb',[-1 -1],'ub',[1 1]); %Polyhedron that makes an AP

plot(P);
hold on;
plot(states(:,1),states(:,2),'b*');
axis([-2 2 -2 2]);
legend('Set','States')

figure
plot(1:size(states, 1), states(:, 1), 'b', 'linewidth', 2);
xlabel('time samples');
ylabel('X coordinate');
%% Ex 1
I = 0:24;
J = 0:5;
fprintf('always[%i,%i] eventually[%i,%i] BE_IN_SET\n', I(1), I(end), J(1), J(end));
C = 100; % smooth constant

[robustness, robustness_smooth] = alwEv(states, P, I, J, C);
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 

%% Ex 2
I = 0:24;
J = 0:4;
fprintf('always[%i,%i] eventually[%i,%i] BE_IN_SET\n', I(1), I(end), J(1), J(end));
C = 100; % smooth constant

[robustness, robustness_smooth] = alwEv(states, P, I, J, C);
fprintf('actual robustness: %.5f \n', robustness);
fprintf('smooth robustness: %.5f \n', robustness_smooth); 


