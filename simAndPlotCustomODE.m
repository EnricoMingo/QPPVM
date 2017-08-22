%This is the main simulation script where is possible to select the control type to run

close all
clear all
clc

% Declare serial manipulator
mdl_puma560_custom
robot = p560;

dt = 0.002;
Tsim = 5;   % Simulation time
qn(3) = -qn(3); %Third joint in home configuration is outside joint limits!
q0 = qn;    % Initial config
qdot0 = q0*0;   % Initial velocity

% I/O handler
io = ControllerIO();

w0 = sqrt(det(robot.jacob0(q0)*robot.jacob0(q0)'));

% Initial configuration inertia matrix
io.Data.w0 = w0;
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

io.Data.Mtrace = [];
io.Data.M_trace = [];


% With QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1, q0, qdot0, io);

% With QP reformulated
% [t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1b, q0, qdot0, io);

% With QP reformulated, qpOASES solver
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1b_qpOASES, q0, qdot0, io);

% With QP reformulated, qpOASES solver
[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_1b_qpOASES_dyn, q0, qdot0, io);

%Without QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_2b, q0, qdot0, io);

%Without QP
%[t, q, qdot, tau] = customDynamicsIntegration(robot.nofriction(), Tsim, dt, @control_law_0, q0, qdot0, io);


% Compute FK
p = p560.fkine(q);

% Plot
figure
plot(t, squeeze(p(1:3,4,:)),'LineWidth', 2.); hold on;
plot(t, io.Data.xref, '--', 'LineWidth', 2.);
xlabel('Time [s]','FontSize',15)
ylabel('Cartesian position [m]','FontSize',15)
AX = legend({'x', 'y', 'z'});
set(AX,'FontSize',15)
set(gca,'FontSize',15)

figure
x1_error = io.Data.xref(3,:) - squeeze(p(3,4,:))';
x2_error = io.Data.xref(1:2,:)' - squeeze(p(1:2,4,:))';
plot(t, [x2_error x1_error'],'LineWidth', 2.);
xlabel('Time [s]','FontSize',15)
ylabel('Cartesian position error [m]','FontSize',15)
AX = legend({'x', 'y', 'z'});
set(AX,'FontSize',15)
set(gca,'FontSize',15)

figure
plot(t, tau,'LineWidth', 2.)
xlabel('Time [s]','FontSize',15)
ylabel('Joint Torques [Nm]','FontSize',15)
set(gca,'FontSize',15)

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
