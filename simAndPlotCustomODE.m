close all
clear all
clc

% Declare serial manipulator
mdl_puma560_custom
robot = p560;

dt = 0.002;
Tsim = 0.8;   % Simulation time
qn(3) = -qn(3); %Third joint in home configuration is outside joint limits!
q0 = qn*1.1;  % Initial config
qdot0 = q0*0;   % Initial velocity

% I/O handler
io = ControllerIO();

% Initial configuration inertia matrix
io.Data.B0 = robot.inertia(q0);
io.Data.q0 = q0;
io.Data.J_previous = robot.jacob0(q0);
io.Data.JacobRank = [];
io.Data.t_previous = 0;
io.Data.xref = [];
io.Data.dt = dt;
io.Data.fval1 = [];
io.Data.fval2 = [];
io.Data.fval3 = [];
io.Data.e1 = [];
io.Data.e2 = [];
io.Data.normM1pinv = [];
io.Data.normA1pinv = [];
io.Data.rankM1 = [];
io.Data.rankA1 = [];
io.Data.rankA2 = [];

% With QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1, q0, qdot0, io);

% With QP reformulated
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1b, q0, qdot0, io);

% With QP reformulated, qpOASES solver
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1b_qpOASES, q0, qdot0, io);

%Without QP
[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_2b, q0, qdot0, io);

%Without QP
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_0, q0, qdot0, io);


% Compute FK
p = p560.fkine(q);

% Plot
figure
plot(t, squeeze(p(1:3,4,:))); hold on;
plot(t, io.Data.xref, '--');
xlabel('Time [s]')
ylabel('Cartesian position [m]')
legend({'x', 'y', 'z', 'x ref', 'y ref', 'z ref'});

figure
x1_error = io.Data.xref(3,:) - squeeze(p(3,4,:))';
x2_error = io.Data.xref(1:2,:)' - squeeze(p(1:2,4,:))';
plot(t, [x2_error x1_error']);
xlabel('Time [s]')
ylabel('Cartesian position error [m]')
legend({'x', 'y', 'z'});

figure
plot(t, tau)
xlabel('Time [s]')
ylabel('Joint Torques [Nm]')

qmin = robot.qlim(:,1);
qmax = robot.qlim(:,2);
qmin = qmin';
qmax = qmax';
qmin = repmat(qmin, length(q),1);
qmax = repmat(qmax, length(q),1);

figure
for i = 1:1:6
    subplot(3,2,i)
    plot(t, q(:,i)); hold on; plot(t, qmin(:,i), '--'); hold on; plot(t, qmax(:,i), '--');
    xlabel('Time [s]')
    ylabel('Joint Positions [rad]')
end

figure
robot.plot(q(1:10:end,:))
