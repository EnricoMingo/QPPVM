function tau = control_law_1(robot, t, q, qdot)

t

period = 2;
amplitude = 0.3;
Kp1 = 15000;
Kd1 = 300;
Kp2 = 15000;
Kd2 = 400;
Tq = robot.fkine(q);
J = robot.jacob0(q);
B = robot.inertia(q);
Binv = inv(B);
taumax = [1 1 1 1 1 1]'*100;
taumin = -taumax;
opt = optimoptions('quadprog', 'Display', 'off');

x1ref = amplitude*sin(2*pi/period*t);
x1 = Tq(3,4);
J1 = J(3,:);
B1 = inv(J1*Binv*J1');
J1pinv = Binv*J1'*B1;
x1dot = J1*qdot';
f1 = -Kp1*(x1-x1ref) - Kd1*x1dot;

x2ref = [0.2 0.2]';
x2 = Tq(1:2,4);
J2 = J(1:2,:);
B2 = inv(J2*Binv*J2');
J2pinv = Binv*J2'*B2;
x2dot = J2*qdot';
f2 = -Kp2*(x2-x2ref) - Kd2*x2dot;

Q1 = J1pinv*J1pinv';
c1 = -f1'*J1pinv';
A1 = [];
b1 = [];
tau1 = quadprog(Q1,c1,[],[],A1,b1,taumin, taumax,[],opt);


Q2 = J2pinv*J2pinv';
c2 = -f2'*J2pinv';
A2 = J1pinv';
b2 = J1pinv'*tau1;
tau1 = quadprog(Q2,c2,[],[],A2,b2,taumin, taumax,[], opt);
f2_opt = J2pinv'*tau1;

Q3 = eye(6);
c3 = zeros(6,1);
A3 = [J1pinv'; J2pinv'];
b3 = A3*tau1;
tau1 = quadprog(Q3,c3,[],[],A3,b3, taumin, taumax, [], opt);


tau = tau1';







end

