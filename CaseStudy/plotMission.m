function rob = plotMission(handles)

w_opt = handles.myhandle.w_opt;
optParams = handles.myhandle.optParams;
time_print = handles.timeElapsed_data;

disp('Plotting...');

[negative_rob, xx,yy,zz] = Mission_Robustness_exact(w_opt,optParams);
rob = -negative_rob;

set(handles.missionRob_data, 'String', sprintf("%.4f",rob));

mar{1} = 'ko';
mar{2} = 'go';
mar{3} = 'ro';
mar{4} = 'bo';
mar{5} = 'co';
mar{6} = 'mo';
mar{7} = 'yo';
mar{8} = 'rp';
mar{9} = 'gp';
mar{10}= 'mp';
mar{11} = 'bp';
mar{12} = 'yp';

col{1} = 'k';
col{2} = 'g';
col{3} = 'r';
col{4} = 'b';
col{5} = 'c';
col{6} = 'm';
col{7} = 'y';
col{8} = 'r';
col{9} = 'g';
col{10}= 'm';
col{11} = 'b';
col{12} = 'y';

p = [];
% Plot Path
for d = 1:optParams.N_drones
    
    hold on;
    p(d) = plot3(xx(:,d),yy(:,d),zz(:,d),'-.','linewidth',3, 'Color', col{d}, 'DisplayName', strcat('Drone',num2str(d)));
%     hold all;
end


for t = 1:size(xx,1)
    if(exist('gc','var'))
        delete(gc)
    end
    for d = 1:optParams.N_drones
        
        hold on;
        gc(d) = plot3(xx(t,d),yy(t,d),zz(t,d),mar{d},'MarkerSize',20, 'MarkerFaceColor', col{d});
    end
    set(time_print, 'String', sprintf("%.3f",(t-1)*optParams.sampling_time));
    pause(optParams.sampling_time);
end
