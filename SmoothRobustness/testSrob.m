clc;close all;clear all;

% be in this files folder
addpath('basic')
addpath('../MiscFunctions/'); %for smoothmin/max
%% test some basic smooth robustness

states = [2*ones(50,2);0*ones(50,2)]; %100 time steps, 2 dimensions

P = Polyhedron('lb',[-1 -1],'ub',[1 1]); %Polyhedron that makes an AP

plot(P);
hold on;
plot(states(:,1),states(:,2),'b*');
axis([-2 2 -2 2]);
legend('Set','States')
%% test eventually_I states in P
disp('Eventually')
C = 10; %constant for smooth max/min approxs, try increasing for accuracy
I = 50:75; %between steps 50 and 75

[rho_e,rho_e_ap] = eventually_I(states,P,I,C);

fprintf('smooth robustness: %.5f \n', rho_e); 
fprintf('actual robustness: %.5f \n', 1); %only for the default data


figure;
plot(rho_e_ap);title('robustness of (state in P) at each time step in I');grid on;


%% test always_I states in P
disp('Always')
C = 100; %constant for smooth max/min approxs, play around with it to see accuracy of the SmoothMin/Max
I = 1:100; %between steps 1 and 100

[rho_a,rho_a_ap] = always_I(states,P,I,C);

fprintf('smooth robustness: %.5f \n', rho_a);
fprintf('actual robustness: %.5f \n', -1); %only for the default data


figure;
plot(rho_a_ap);title('robustness of (state in P) at each time step in I');grid on;
