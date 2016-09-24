function tau = control_law_2(robot, t, q, qdot, io)
%% control_law_2
% Uses
%      tau = J'f
% to find joint torques that realizes a certain Cartesian Force 
% at the end-effector considering also low priority tasks (using
% the classical Null-Space Projection.

%% Actual information from the robot
Tq = robot.fkine(q);
J = robot.jacob0(q);
B = robot.inertia(q);
%B = eye(6,6);
Binv = inv(B);

%% Reference Trajectory for Main Task (the main task is a Cartesian Position Trj Task)
period = 2;
amplitude = 0.3;
x1ref = amplitude*sin(2*pi/period*t);

Kp1 = 15000;
Kd1 = 300;
x1 = Tq(3,4);
J1 = J(3,:);
B1 = inv(J1*Binv*J1');
J1pinv = Binv*J1'*B1;
x1dot = J1*qdot';
f1 = -Kp1*(x1-x1ref) - Kd1*x1dot;

%% Reference for Secondary task (the secondary task is a Cartesian Position Task)
Kp2 = 15000;
Kd2 = 400;

x2ref = [0.2 0.2]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
B2 = inv(J2*Binv*J2');
J2pinv = Binv*J2'*B2;
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

xref = [x2ref; x1ref];
io.Data.xref = [io.Data.xref xref];

P1 = (eye(robot.n) - J1'*J1pinv'); % First task nullspace projector
TMP = J2pinv'*P1;
tau0 = pinv(TMP)*(f2-J2pinv'*J1'*f1);

tau = J1'*f1 + P1*tau0;


tau = tau';


if mod(t,1) == 0
    t
end


end

