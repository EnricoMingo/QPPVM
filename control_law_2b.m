function tau = control_law_2b(robot, t, q, qdot, io)
%% control_law_2b
% Computes a closed form solution of a two-tasks problem
% in a very disputable way


%% Actual information from the robot
Tq = robot.fkine(q); % pose
J = robot.jacob0(q); % Jacobian
io.Data.J_previous = J;
io.Data.t_previous = t;
io.Data.JacobRank = [io.Data.JacobRank rank(J)];


B = robot.inertia(q); % Inertia
%B = eye(6,6);
Binv = inv(B);
taumax = [1 1 1 1 1 1]'*100; % Max allowed torques
taumin = -taumax; % Min allowed torques
if t == 0
    io.Data.taumax = taumax;
    io.Data.taumin = taumin;
end

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
Kd2 = 300;

x2ref = [1. 1.]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

xref = [x2ref; x1ref];
io.Data.xref = [io.Data.xref xref];

%% Analytic solution (two tasks) TODO: properly computed n-tasks

A1 = J1*Binv;
b1 = J1*Binv*J1'*f1;

A2 = J2*Binv;
b2 = J2*Binv*J2'*f2;

% % Solve KKT system
% A_KKT = [A2'*A2 + 0.001*eye(6) A1'; A1 zeros(size(A1,1))];
% b_KKT = [A2'*b2; b1];
% solution = A_KKT\b_KKT;
% tau1 = solution(1:6);

P1 = eye(6) - A1\A1;
M1 = A2*P1;
% M1pinv = M1'*inv(M1*M1' + 0.001*eye(size(M1,1)));
tau0 = M1\(b2-A2*(A1\b1));
tau1 = A1\b1 + P1*tau0;

io.Data.e1 = [io.Data.e1 J1*Binv*tau1-J1*Binv*J1'*f1];
io.Data.e2 = [io.Data.e2 J2*Binv*tau1-J2*Binv*J2'*f2];

% io.Data.normA1pinv = [io.Data.normA1pinv norm(A1pinv)];
% io.Data.normM1pinv = [io.Data.normM1pinv norm(A2pinv)];
% % io.Data.rankM1 = [io.Data.rankM1 rank(M1)];
% io.Data.rankA1 = [io.Data.rankA1 rank(A1)];
% io.Data.rankA2 = [io.Data.rankA2 rank(A2)];


% for i = 1:1:length(tau1)
%     if tau1(i) >= 100
%         tau1(i) = 100;
%     elseif tau1(i) <= -100
%         tau1(i) = -100;
%     end
% end

tau = tau1';

io.Data.tau = tau;



if mod(t*10,1) == 0
    t
end



end

