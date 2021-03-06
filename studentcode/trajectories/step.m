function [desired_state] = step(t)

% Step params
x0 = 1;
y0 = 1;
z0 = 0;
yaw0 = 0;

%% You should NOT need to change anything below this line %%%%%%%%%%%%%%%%%
% Set the position based on time
if t == 0
    pos = [0 0 0]'; % start at the origin
    yaw = 0;
else
    pos = [x0 y0 z0]'; % goal location
    yaw = yaw0;
end

% Set the velocities to be 0
vel = [0 0 0]';
acc = [0 0 0]';
yawdot = 0;

% Set the desired state (as a struct)
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
