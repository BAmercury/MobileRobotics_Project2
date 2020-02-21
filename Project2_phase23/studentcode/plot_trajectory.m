function plot_trajectory(map, path)
[DT, ~] = computeCoefficients(map, path, crazyflie);

T_final = sum(DT);
t = 0 : 0.01 : T_final;
twp = [0, cumsum(DT)]; % time for each waypoint

pos = zeros(3, length(t)); % initialize data arrays
vel = zeros(3, length(t));
acc = zeros(3, length(t));
yaw = zeros(1, length(t));
yawdot = zeros(1, length(t));

% Initialize trajectory_generator
trajectory_generator([], map, path, crazyflie);
for i = 1:length(t)
    desired_state = trajectory_generator(t(i)); % get desired state at each time
    pos(:,i) = desired_state.pos; % fill in data arrays
    vel(:,i) = desired_state.vel;
    acc(:,i) = desired_state.acc;
    yaw(i) = desired_state.yaw;
    yawdot(i) = desired_state.yawdot;
end

% Plot polynomial trajectory in map
figure(1)
hold on
plot3(pos(1,:), pos(2,:), pos(3,:), 'k-', 'LineWidth', 3);
hold off
legend('Blocks', 'A* path', 'Start', 'Goal', 'Polynomial Trajectory')

% Plot positions
figure(2)
subplot(3,1,1)
plot(t, pos(1,:), 'k', twp, path(1,:), 'ro')
title('Positions')
legend('Polynomial Trajectory', 'Waypoints', 'Location', 'Best')
xlabel('Time [s]'); ylabel('x position [m]');
subplot(3,1,2)
plot(t, pos(2,:), 'k', twp, path(2,:), 'ro')
xlabel('Time [s]'); ylabel('y position [m]');
subplot(3,1,3)
plot(t, pos(3,:), 'k', twp, path(3,:), 'ro')
xlabel('Time [s]'); ylabel('z position [m]');

% Plot velocities
figure(3)
subplot(3,1,1)
plot(t, vel(1,:), 'k')
title('Linear Velocities')
xlabel('Time [s]'); ylabel('x velocity [m]');
subplot(3,1,2)
plot(t, vel(2,:), 'k')
xlabel('Time [s]'); ylabel('y velocity [m]');
subplot(3,1,3)
plot(t, vel(3,:), 'k')
xlabel('Time [s]'); ylabel('z velocity [m]');

% Plot velocities
figure(4)
subplot(3,1,1)
plot(t, acc(1,:), 'k')
title('Linear Accelerations')
xlabel('Time [s]'); ylabel('x acceleration [m]');
subplot(3,1,2)
plot(t, acc(2,:), 'k')
xlabel('Time [s]'); ylabel('y acceleration [m]');
subplot(3,1,3)
plot(t, acc(3,:), 'k')
xlabel('Time [s]'); ylabel('z acceleration [m]');

% Plot angles
figure(5)
subplot(2,1,1)
plot(t, yaw, 'k')
xlabel('Time [s]'); ylabel('yaw [radians]');
title('Yaw and Yaw Velocity')
subplot(2,1,2)
plot(t, yawdot, 'k')
xlabel('Time [s]'); ylabel('yaw velocity [radians/s]');
