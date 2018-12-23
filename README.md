# FlyByLogic

See documentation in the manual folder for details on the user interface. 

Requirements:
MPT 3.+ toolbox: http://people.ee.ethz.ch/~mpt/3/
CVX: http://cvxr.com/cvx/download/
Casadi MATLAB: https://github.com/casadi/casadi/wiki/matlab
HSL routines for ipopt (optional, to get best performance):  http://www.hsl.rl.ac.uk/ipopt/

Includes code to run simulation examples from Pant et al.,FlyByLogic, ICCPS 2018:

Run the RunAllExamples.m file. Edit parameters in individual files called in it to run different cases. If you have the hsl routines, change the solver from mumps to ma27 in the solver options for the individual files.


