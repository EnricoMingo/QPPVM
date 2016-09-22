clear all
clc

% Declare serial manipulator
mdl_puma560_custom
robot = p560;

Tsim = 5;   % Simulation time
q0 = qn;    % Initial config
qdot0 = q0*0;   % Initial velocity

% With QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1, q0, qdot0);

% With QP reformulated
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1b, q0, qdot0);

% With QP reformulated, qpOASES solver
[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1b_qpOASES, q0, qdot0);

%Without QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_2, q0, qdot0)

% Compute FK
p = p560.fkine(q);

% Plot
figure
plot(t, squeeze(p(1:3,4,:)))
xlabel('Time [s]')
ylabel('Cartesian position [m]')
legend({'x', 'y', 'z'});

figure
subplot(2,1,1)
plot(t, tau)
xlabel('Time [s]')
ylabel('Joint Torques [Nm]')

subplot(2,1,2)
plot(t, q)
xlabel('Time [s]')
ylabel('Joint Positions [rad]')

figure
robot.plot(q)
