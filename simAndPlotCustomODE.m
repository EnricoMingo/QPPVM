clear all
clc

% Declare serial manipulator
mdl_puma560_custom
robot = p560;

Tsim = 5;   % Simulation time
q0 = qn;    % Initial config
qdot0 = q0*0;   % Initial velocity

% With QP
[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1, q0, qdot0);

% With QP reformulated
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1b, q0, qdot0);

%Without QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_2, q0, qdot0);

% Compute FK
p = p560.fkine(q);

% Plot
figure
plot(t, squeeze(p(1:3,4,:)))
xlabel('Time [s]')
ylabel('Cartesian position [m]')
legend({'x', 'y', 'z'});

figure
t = 0:0.002:Tsim;
period = 2;
amplitude = 0.3;
x1ref = amplitude*sin(2*pi/period*t);
x1_error = x1ref - squeeze(p(3,4,:))';
x2ref = [0.2 0.2]';
x2_error = repmat(x2ref',length(squeeze(p(1:2,4,:))'),1) - squeeze(p(1:2,4,:))';
plot(t, [x2_error x1_error']);
xlabel('Time [s]')
ylabel('Cartesian position error [m]')
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
