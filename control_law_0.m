function tau = control_law_0( robot, t, q, qdot, io )
%CONTROL_LAW_0 Summary of this function goes here
%   Detailed explanation goes here

%% Actual information from the robot
Tq = robot.fkine(q);
J = robot.jacob0(q);

%% Reference Trajectory for Main Task (the main task is a Cartesian Position Trj Task)
period = 2;
amplitude = 0.3;
x1ref = amplitude*sin(2*pi/period*t);

Kp1 = 15000;
Kd1 = 300;
x1 = Tq(3,4);
J1 = J(3,:);
x1dot = J1*qdot';
f1 = -Kp1*(x1-x1ref) - Kd1*x1dot;

%% Reference for Secondary task (the secondary task is a Cartesian Position Task)
Kp2 = 15000;
Kd2 = 400;

x2ref = [1.0 1.0]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

xref = [x2ref; x1ref];
io.Data.xref = [io.Data.xref xref];


tau = J(1:3,:)'*[f2;f1];


tau = tau';


if mod(t,1) == 0
    t
end


end


