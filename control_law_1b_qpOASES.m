function tau = control_law_1b_qpOASES(robot, t, q, qdot, io)
%% control_law_1b 
% Uses a QP to solve the problem to find joint torques that realizes a
% certain Cartesian Force at the end-effector considering also low priority
% tasks and inequality constraints.
%
% Note that the quadratic cost function is:
%     F = ||JB^-1*tau - JB^1-J'f||
% with pinv_J1_B the dynamically consistent pseudo-inverse of J,
% and the Optimality conditions for low priority tasks is:
%     JB^-1*tau = JB^-1*tau_
% where tau_ = argmin F of the previous task.


%% Actual information from the robot
Tq = robot.fkine(q); % pose
J = robot.jacob0(q); % Jacobian
B = eye(6,6);
%B = robot.inertia(q); % Inertia
%C = robot.coriolis(q,qdot)*qdot';
%g = robot.gravload(q);
Binv = inv(B); 

taumax = [1 1 1 1 1 1]'*100;% Max allowed torques
taumin = -taumax; % Min allowed torques
if t == 0
    io.Data.taumax = taumax;
    io.Data.taumin = taumin;
end
umin = taumin;
umax = taumax;


%% Reference Trajectory for Main Task (the main task is a Cartesian Position Trj Task)
period = 2;
amplitude = 0.3;
x1ref = amplitude*sin(2*pi/period*t);

Kp1 = 15000;
Kd1 = 300;

x1 = Tq(3,4);
J1 = J(3,:);
x1dot = J1*qdot';
%Jdot = robot.jacob_dot(q,qdot);
%u = (Binv*J1'*inv(J1*Binv*J1'))'*C-inv(J1*Binv*J1')*Jdot(3,:);
%p = (Binv*J1'*inv(J1*Binv*J1'))'*g';
f1 = -Kp1*(x1-x1ref) - Kd1*x1dot;% + u + p;


%% Reference for Secondary task (the secondary task is a Cartesian Position Task)
Kp2 = 15000;
Kd2 = 400;

%x2ref = [0.2 0.2]';
x2ref = [1.0 1.0]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

xref = [x2ref(1:2); x1ref];
io.Data.xref = [io.Data.xref xref];

%% Solution using QP
options = qpOASES_options( 'MPC' );
%options = qpOASES_options( 'reliable' );
%options = qpOASES_options( 'default' );

% Solution of Main Task
Q1 = Binv*J1'*J1*Binv;
c1 = -Binv*J1'*J1*Binv*J1'*f1;
A1 = [];
b1 = [];
[tau1,fval,exitflag,iter,lambda,auxOutput] = qpOASES(Q1,c1,[],umin,umax,[],[], options);
io.Data.fval1 = [io.Data.fval1 fval];


% % Solution of the Secondary Task
Q2 = Binv*J2'*J2*Binv;
c2 = -Binv*J2'*J2*Binv*J2'*f2;
A2 = J1*Binv; % Optimality condition
b2 = J1*Binv*tau1; % Optimality condition
[tau1,fval,exitflag,iter,lambda,auxOutput] = qpOASES(Q2,c2,A2,umin,umax,b2,b2, options);
io.Data.fval2 = [io.Data.fval2 fval];

% % Solution of a Third Task in Joint space (Joint Torque minimzation)
% K = 1000;
% D = 100;
% tau0 = K*(zeros(1,6)-q)-D*qdot;
% Q3 = eye(6)*Binv;
% c3 = -tau0*Binv;
% A3 = [J1*Binv; J2*Binv]; %Optimality Condition
% b3 = A3*tau1; %Optimality Condition
% [tau1,fval,exitflag,iter,lambda,auxOutput] = qpOASES(Q3,c3',A3,umin,umax,b3,b3, options);
% io.Data.fval3 = [io.Data.fval3 fval];

tau = tau1';% + C' + g;



if mod(t,1) == 0
    t
end



end

