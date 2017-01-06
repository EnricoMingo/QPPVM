Multi-priority Cartesian Impedance Control based on Quadratic Programming Optimization
--------------------------------------------------------------------------------------
This repository contains a prototype of a Multi-Priority Cartesian Imepdance Controller based on Quadratic Programming Optimization. Results are shown using the simlated version of a modified Puma560 manipulator. We consider three Cartesian tasks:
- reaching 1.0 m along the x axis
- reaching 1.0 m along the y axis
- following a sinusoidal trajectory, of amplitude 0.3 m, around the setpoint 0.0 m along the z axis

We consider the task along z of higher priority wrt the other two. We show how our QP formulation is superior wrt the classical formulation considering also constraints such as joint torque limits.

How to run the simulations:
---------------------------
To run the examples you need:
- The Robotics Toolbox for Matlab, by Peter Corke that you can find <a href="http://www.petercorke.com/Robotics_Toolbox.html" target="target">here</a>
- The QP solver qpOASES, by H.J. Ferreau et al., that you can find <a href="https://projects.coin-or.org/qpOASES" target="target">here</a>

Just run ```simAndPlotCustomODE.m``` to run the simulation. To change controller you need to uncomment the desired controller inside ```simAndPlotCustomODE.m```.
