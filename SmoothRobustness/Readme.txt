Quick and dirty matlab implementation to get some smooth robustness

needs the MPT3+ toolbox from ethz

folder basic has the code for smooth robustness of 
1. always_I (x \in P), 2. eventually_I (x \in P), 3. always_I(not x\in P)

Atomic propositions are of the from x \in P, where P is a polyhedron, i.e. Hx<=g.
The associated robustness is min_i (g_i - H_i*x). 

Other atomic propositions can be implemented by swapping the code (in basic) for Hx<=g and the min after that with your favourite function.

other files:

testSrob.m: some basic tests on the always and eventually code. Start has

ReachAvoidExample.m: get robustness of a trajectory that reaches a goal set while avoiding an obstacle.