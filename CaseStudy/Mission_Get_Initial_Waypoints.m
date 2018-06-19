function [ww0,vv0] = Mission_Get_Initial_Waypoints(init_state,optParams,mission_type)

% Extract parameters from optParams
H = optParams.H_formula;
M1 = optParams.M1;
T = optParams.T;
K1_T = optParams.K1_T;
K2_tprime = optParams.K2_tprime;
max_vel=optParams.max_vel;
max_accl=optParams.max_accl;
da = optParams.da;

% Randomize (If more than 2 drones)
randomize = (optParams.N_drones>2);

% CVX_solver sedumi
cvx_begin quiet

variable ww(H+1,3)
variable vv(H+1,3)

minimise sum(ww(:,1)) %sum(ww(:,1)) makes opt fast, 0 works too

ww(1,:) == init_state(1:3)'; %#ok<*NODEF,*EQEFF,*VUNUS>
vv(1,:) == init_state(4:6)';

for i = 2:H+1
    
    ww(i,:)>=optParams.map.boundary(1:3);
    ww(i,:)<=optParams.map.boundary(4:6);
    for j = 1:3
        
        al = M1(1,:)*[(ww(i,j)-ww(i-1,j)-vv(i-1,j)*T);0;da];
        be = M1(2,:)*[(ww(i,j)-ww(i-1,j)-vv(i-1,j)*T);0;da];
        gam = M1(3,:)*[(ww(i,j)-ww(i-1,j)-vv(i-1,j)*T);0;da];
        
        %enforce w dynamics
        % ww(i,j) == (al/120)*T^5 + (be/24)*T^4 + (gam/6)*T^3 + vv(i-1)*T + ww(i-1,j);
        K1_T*(ww(i,j)-ww(i-1,j))+(1-T*K1_T)*vv(i-1,j)<=max_vel;
        K1_T*(ww(i,j)-ww(i-1,j))+(1-T*K1_T)*vv(i-1,j)>=-max_vel;
        K2_tprime*(ww(i,j)-ww(i-1,j))-T*K2_tprime*vv(i-1,j)<=max_accl;
        K2_tprime*(ww(i,j)-ww(i-1,j))-T*K2_tprime*vv(i-1,j)>=-max_accl;
        
        %enforce v
        vv(i,j) == (al/24)*T^4 + (be/6)*T^3 + (gam/2)*T^2 + vv(i-1,j);
        vv(i,j) <= max_vel;
        vv(i,j) >= -max_vel;        
   
    end
end

% if(mission_type==1) %package delivery
%     ww(ceil((H+1)/2),:) == optParams.goal{1}.stop' + (0.2*rand(3,1)' - 0.1)*randomize; %deliver
%     ww(H+1,:) ==  optParams.goal{4}.stop' + (0.2*rand(3,1)' - 0.1)*randomize; % go to base
% else
%     ww(ceil(1*(H+1)/4),:) == optParams.goal{2}.stop' + (0.2*rand(3,1)' - 0.1)*randomize;
%     ww(ceil(3*(H+1)/4),:) == optParams.goal{2}.stop' + (0.2*rand(3,1)' - 0.1)*randomize;
%     ww(ceil(2*(H+1)/4),:) == optParams.goal{3}.stop' + (0.2*rand(3,1)' - 0.1)*randomize;
%     ww(ceil(4*(H+1)/4),:) == optParams.goal{3}.stop' + (0.2*rand(3,1)' - 0.1)*randomize; 
% end

cvx_end

ww0 = reshape(ww',(H+1)*3,1);
vv0 = reshape(vv',(H+1)*3,1);
