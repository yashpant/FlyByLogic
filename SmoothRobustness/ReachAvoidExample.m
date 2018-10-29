% see if a robot makes it to a goal set between time steps 150 to 200 while
% avoiding an unsafe region
clc;close all;clear all;
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
%% make a map
Obstacle = Polyhedron('lb',[-1 -1],'ub',[1 1]); %Box like poly
Obstacle.minHRep;
Goal = Polyhedron('A',[0 -1],'b',-1.5); %Ax<=b form. Wants x2>=1.5, x1 is free
% Goal set is simply the second coordinate being above 1.5.
Goal.minHRep; %need H-rep for getting poly in form Ax<=b

figure(1)
plot(Goal,'alpha',0.5,'color','green');
hold on;
plot(Obstacle,'alpha',0.5,'color','red');

legend('obstacle','goal');
axis([-3 3 -3 3])

%% make a trajectory of 200 time steps

Y_segment1 = linspace(-2,1.75,100)';
X_segment1 = -2*ones(size(Y_segment1));

X_segment2 = linspace(-2,1.75,100)';
Y_segment2 = 1.75*ones(size(X_segment2));

states = [[X_segment1;X_segment2] [Y_segment1;Y_segment2]]; % the trajectory

plot(states(:,1),states(:,2),'.')
legend('obstacle','goal','states');

%% get robustness
I_goal = 150:200;
%Goal = Polyhedron('lb',[1.5 1.5],'ub',[2 2]);
[rho,rho_goal,rho_obstacle] = robustness_reachavoid(states,Goal,Obstacle,I_goal);

fprintf('Overall smooth robustness: %.5f \n', rho);
fprintf('Eventually_I goal smooth robustness: %.5f \n', rho_goal);
fprintf('Always_ not in Obstacle smooth robustness: %.5f \n', rho_obstacle);
fprintf('Real robustness was 0.25,0.25,0.75 respectively \n'); %for original data


