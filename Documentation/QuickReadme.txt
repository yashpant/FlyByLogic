Requirements (MATLAB version): 
1. MPT 3.+ toolbox: http://people.ee.ethz.ch/~mpt/3/ 
2. CVX: http://cvxr.com/cvx/download/ 
3. Casadi MATLAB binaries: https://github.com/casadi/casadi/wiki/InstallationInstructions

For installing the C++ version, see Instructions.md in the Documentation folder.

For the GUI:
1. Navigate to the Gui folder. 
2. Run Mission_Gui.m
3. Select a pre-made mission (like patrol.mat) and click on Load Mission. 
4. Plot it (Plot Mission).
5. Edit it/make a new mission and press "Plan Mission" to compute trajectories. Try both the MATLAB and C++ solvers.
6. Inside Mission_Gui.m, track the flow of information to see what is being called.

Easy to use code:
1. Navigate to the FlyByLogic/variable_vel_trajs folder
2. Open the reach_avoid_Ndrones_varvel.m file. The scenarios is 2 drones doing an timed reach-avoid mission. You can change the number of drones by changing the N_drones variable.
3. Go through it to see what's going on inside. 

C++ Code:
1. It's in the AATC_cpp folder
2. AATC.h has the function declarations for the all methods, AATC.cpp has the implementation.
3. See the main.cpp and Fbl_gui_interface file as well to see how it talks to Matlab.
