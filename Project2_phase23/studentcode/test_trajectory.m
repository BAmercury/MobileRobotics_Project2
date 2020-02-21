function [xtraj, ttraj, terminate_cond] = test_trajectory(start, stop, map, path, vis)
% TEST_TRAJECTORY simulates the robot from START to STOP following a PATH
% that's been planned for MAP.
% start - a 3d vector or a cell contains multiple 3d vectors
% stop  - a 3d vector or a cell contains multiple 3d vectors
% map   - map generated by your load_map
% path  - n x 3 matrix path planned by your dijkstra algorithm
% vis   - true for displaying visualization

%Controller and trajectory generator handles
controlhandle = @controller;
trajhandle    = @trajectory_generator;

% Make column vector
start = start(:);
stop = stop(:);

% Quadrotor model
params = crazyflie();

% Initialize trajectory
trajhandle([], map, path, params);

%% **************************** FIGURES *****************************
% Environment figure
if nargin < 5
    vis = true;
end

fprintf('Initializing figures...\n')
if vis
    figure('Name', 'Environment');
else
    figure('Name', 'Environment', 'Visible', 'Off');
end
plot_path(map, path);
h_3d = gca;
drawnow;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);
set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol  = 90;         % maximum simulation time
starttime = 0;          % start of simulation in seconds
tstep     = 0.01;       % this determines the time step at which the solution is given
cstep     = 0.05;       % image capture time interval
nstep     = cstep/tstep;
time      = starttime;  % current time
max_iter  = time_tol / cstep;      % max iteration

% Get start and stop position
x0    = init_state(start, 0);
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);

% Maximum position error of the quadrotor at goal
pos_tol  = 0.05; % m
% Maximum speed of the quadrotor at goal
vel_tol  = 0.05; % m/s

x = x0;        % state

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....\n')
for iter = 1:max_iter
    timeint = time:tstep:time+cstep;
    tic;
    % Initialize quad plot
    if iter == 1
        QP = QuadPlot(x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
        desired_state = trajhandle(time);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end

    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
    x = xsave(end, :)';
    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep)   = tsave(1:end-1);

    % Update quad plot
    desired_state = trajhandle(time + cstep);
    QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);

    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    time = time + cstep; % Update simulation time
    t = toc;

    % Pause to make real-time
    if (t < cstep)
        pause(cstep - t);
    end
    
    % Check for collisions with obstacles
    if env_collision_check(x, map.bound_xyz, map.blocks, params)
        error('The robot collided with an obstacle or left the environment');
    end

    % Check termination criteria
    terminate_cond = terminate_check(x, time, stop, pos_tol, vel_tol, time_tol);
    if terminate_cond
        break
    end

end

fprintf('Simulation Finished....\n')

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);

% Plot the saved position and velocity of each robot
if vis
    % Truncate saved variables
    QP.TruncateHist();
    % Plot position for each quad
    h_pos = figure('Name', 'Quad position');
    plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
    plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
    % Plot velocity for each quad
    h_vel = figure('Name', 'Quad velocity');
    plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
    plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');
end

end
