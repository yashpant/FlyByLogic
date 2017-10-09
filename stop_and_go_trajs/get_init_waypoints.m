function w0 = get_init_waypoints(p0,optParams)
%%
H = optParams.H_formula;

cvx_solver sedumi
cvx_begin quiet

variable ww(H+1,3)
variable v(H,3)

minimise sum(ww(:,1)) %sum(ww(:,1)) makes opt fast, 0 works too

ww(1,:) == p0';
for i = 2:H+1
    ww(i,:) == ww(i-1,:) + v(i-1,:);
    ww(i,:)>=optParams.map.boundary(1:3);
    ww(i,:)<=optParams.map.boundary(4:6);
    v(i-1,:)<=optParams.max_per_axis*ones(3,1)';
    v(i-1,:)>=-optParams.max_per_axis*ones(3,1)';
end
ww(H+1,:) == optParams.goal.stop' + (0.2*rand(3,1)' - 0.1)*(optParams.N_drones>1); %peturb a bit
cvx_end

w0 = reshape(ww',(H+1)*3,1);