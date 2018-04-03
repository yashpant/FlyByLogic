# FlyByLogic
Code to run simulation examples from FlyByLogic, ICCPS 2018.

Requirements:
MPT 3.+ toolbox: http://people.ee.ethz.ch/~mpt/3/
CVX: http://cvxr.com/cvx/download/
Casadi MATLAB: https://github.com/casadi/casadi/wiki/matlab
HSL routines for ipopt (optional, to get best performance):  http://www.hsl.rl.ac.uk/ipopt/

Run the RunAllExamples.m file. Edit parameters in individual files called in it to run different cases. If you have the hsl routines, change the solver from mumps to ma27 in the solver options for the individual files.

A GUI based front end to specify generalized reach-avoid missions is under development and will be added here.
