function tau = control_law_1(robot, t, q, qdot, io)
%% control_law_1 
% Uses a QP to solve the problem to find joint torques that realizes a
% certain Cartesian Force at the end-effector considering also low priority
% tasks and inequality constraints.
%
% Note that the quadratic cost function is:
%     F = ||(pinv_J_B)'*tau - f||
% with pinv_J1_B the dynamically consistent pseudo-inverse of J,
% and the Optimality conditions for low priority tasks is:
%     (pinv_J_B)'*tau = (pinv_J_B)'*tau_
% where tau_ = argmin F of the previous task.


%% Actual information from the robot
Tq = robot.fkine(q); % pose
J = robot.jacob0(q); % Jacobian
B = robot.inertia(q); % Inertia
%B = eye(6,6);
Binv = inv(B); 
taumax = [1 1 1 1 1 1]'*100; % Max allowed torques
taumin = -taumax; % Min allowed torques

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

x2ref = [0.2 0.2]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

%% Solution using QP
opt = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');

% Solution of Main Task
B1 = inv(J1*Binv*J1');
J1pinv = Binv*J1'*B1;

Q1 = J1pinv*J1pinv';
c1 = -f1'*J1pinv';
A1 = [];
b1 = [];
tau1 = quadprog(Q1,c1,[],[],A1,b1,taumin, taumax,[],opt);

% Solution of the Secondary Task
B2 = inv(J2*Binv*J2');
J2pinv = Binv*J2'*B2;

Q2 = J2pinv*J2pinv';
c2 = -f2'*J2pinv';
A2 = J1pinv'; % Optimality condition
b2 = J1pinv'*tau1; % Optimality condition
tau1 = quadprog(Q2,c2,[],[],A2,b2,taumin, taumax,[], opt);
%f2_opt = J2pinv'*tau1;

% Solution of a Third Task in Joint space (Joint Torque minimzation)
K = 1000;
D = 100;
tau0 = K*(io.Data.q0-q)-D*qdot;
Q3 = eye(6)*Binv;
c3 = -tau0*Binv;
A3 = [J1pinv'; J2pinv']; %Optimality Condition
b3 = A3*tau1; %Optimality Condition
tau1 = quadprog(Q3,c3,[],[],A3,b3, taumin, taumax, [], opt);

tau = tau1';


if mod(t,1) == 0
    t
end



end

