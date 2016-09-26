function [t, q, qdot, tau] = customDynamicsIntegration(robot, T, dt, TAUFUN, q0, qdot0, io)

t = 0:dt:T; t = t(:);
q(length(t), robot.n) = 0;
qdot(length(t), robot.n) = 0;
tau(length(t), robot.n) = 0;

q(1, :) = q0(:)';
qdot(1,:) = qdot0(:)';

for k = 1:length(t)
    

    tau(k, :) = TAUFUN(robot, t(k), q(k,:), qdot(k,:), io);
    
    if k ~= length(t)
        qddot = robot.accel(q(k,:), qdot(k,:), tau(k, :));
        qdot(k+1, :) = qdot(k, :) + qddot'*dt;
        q(k+1, :) = q(k, :) + qdot(k, :)*dt + 0.5*qddot'*dt^2;
    end
    
end

