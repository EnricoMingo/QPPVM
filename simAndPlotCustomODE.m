clear robot Tsim q0 qdot0 t q qdot p

% Declare serial manipulator
mdl_puma560
robot = p560;

Tsim = 2;   % Simulation time
q0 = qn;    % Initial config
qdot0 = q0*0;   % Initial velocity

% With QP
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_1, q0, qdot0)

Without QP
[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, @control_law_2, q0, qdot0)

% Compute FK
p = p560.fkine(q);

% Plot
figure
plot(t, squeeze(p(1:3,4,:)))
xlabel('Time [s]')
ylabel('Cartesian position [m]')
legend({'x', 'y', 'z'});
