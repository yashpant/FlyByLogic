%% start case study multiquads with eth tracker++

% Clear CMD window and close all figures
clc; close all;

% Add paths to needed files
addpath('../MiscFunctions');
addpath('../Maps_mrsl');
addpath('subfunctions');
%% Define Mission Parameters

disp('Initializing...');

% drone_goals{kth drone} = [goal-index  time-Interval; goal-index time-Interval ]
drone_goals{1} = [1 0 10; 2 10 20; 1 20 30; 1 25 30];
drone_goals{2} = [2 0 30; 3 0 30; 4 0 30];
drone_goals{3} = [1 0 30; 2 0 30];

% Choose your map!
map_name = 'Maps_mrsl/map1.txt';

% Define Goals (Places we want to be at some time)
goal{1}.stop = [4;4;2]; %delivery goal
goal{1}.ds = 0.5; %thickeness
goal{1}.lb = goal{1}.stop-goal{1}.ds;
goal{1}.ub = goal{1}.stop+goal{1}.ds;

goal{2}.stop = [0;0;4]; %surveliance one
goal{2}.ds = 0.5; %thickeness
goal{2}.lb = goal{2}.stop-goal{2}.ds;
goal{2}.ub = goal{2}.stop+goal{2}.ds;

goal{3}.stop = [4;-4;2]; %surveliance two
goal{3}.ds = 0.5; %thickeness
goal{3}.lb = goal{3}.stop-goal{3}.ds;
goal{3}.ub = goal{3}.stop+goal{3}.ds;

goal{4}.stop = [-4;4;1]; %delivery base
goal{4}.ds = 0.5; %thickeness
goal{4}.lb = goal{4}.stop-goal{4}.ds;
goal{4}.ub = goal{4}.stop+goal{4}.ds;

% Define some colors
col{1} = 'blue';col{2} = 'yellow';
col{3} = 'yellow'; col{4} = 'green';

goal{1}.col = col{1};
goal{2}.col = col{2};
goal{3}.col = col{3};
goal{4}.col = col{4};

% Plot Goals and map 
[map, obs] = plot_env(goal, map_name);

missionHandle.N_drones = 3;
missionHandle.d_min = 0.1;
missionHandle.Horizon = 30;
missionHandle.sampling_time = 1/20;
missionHandle.t_step = 1;
missionHandle.map = map;
missionHandle.obs = obs;
missionHandle.goal = goal;
missionHandle.drone_goals = drone_goals;

[w_opt, optParams, time_taken] = planMission(missionHandle);

%%
disp('Plotting...');
if(1)
%     pause;
   
    waypoints = cell(optParams.N_drones,1);
    mar{1} = 'k*';
    mar{2} = 'g*';
    mar{3} = 'r*';
    mar{4} = 'b*';
    mar{5} = 'c*';
    mar{6} = 'm*';
    mar{7} = 'y*';
    mar{8} = 'r*';
    mar{9} = 'g*';
    mar{10} = 'k*';
    mar{11} = 'b*';
    mar{12} = 'c*';

    [negative_rob,xx,yy,zz] = Mission_Robustness(w_opt,optParams);
    
    rob = -negative_rob;
    pause(0.1);
    mar{1} = 'ko';
    mar{2} = 'g*';
    mar{3} = 'r+';
    mar{4} = 'bd';
    mar{5} = 'cs';
    mar{6} = 'mp';
    mar{7} = 'yh';
    mar{8} = 'rp';
    mar{9} = 'g^';
    mar{10}= 'md';
    mar{11} = 'b*';
    mar{12} = 'y*';
    for d = 1:optParams.N_drones
        
        hold on;plot3(xx(:,d),yy(:,d),zz(:,d),'-.','linewidth',0.25);
        hold all;
    end
    for t = 1:size(xx,1)
        if(exist('gc','var'))
                        delete(gc)
        end
        for d = 1:optParams.N_drones
                               
            hold on;
            gc(d) = plot3(xx(t,d),yy(t,d),zz(t,d),mar{d},'MarkerSize',10);
            
        end
        pause(h); 
    end
    
end
 
