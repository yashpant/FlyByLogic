function rob = plotMission(w_opt, optParams)

disp('Plotting...');

[negative_rob,xx,yy,zz] = Mission_Robustness(w_opt,optParams);
rob = -negative_rob;

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

% Plot Path
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
    pause(optParams.sampling_time);
end

 
