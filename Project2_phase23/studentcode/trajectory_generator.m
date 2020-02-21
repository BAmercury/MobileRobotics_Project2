function [ desired_state ] = trajectory_generator(t, map, path, params)
%specify the desired state as a function of time

% Pre-process path
persistent T0 C0 path0;
if nargin > 1
    % The first time through this function will compute the coefficients
    % and store them.  Future times through the function will just call on
    % the saved coefficients.
    [DT, C] = computeCoefficients(map, path, params);

    C0 = C;
    T0 = [0, cumsum(DT)];
    path0 = path;
else
    C = C0;
    T = T0;
    path = path0;
    if size(path,2) == 1
        pos = path(:,end);
        vel = [0;0;0];
    else
        % Straight line
        %%% Linearly interpolate to find position along segment
        idx = find(T > t, 1, 'first') - 1; % get index of current segment
        if isempty(idx)
            idx = length(C);
            q = T(idx + 1) - T(idx);
        else
            q = t - T(idx); % time along current segment
        end
        % Position
        px = C{idx}(:,1); % x coefficients
        py = C{idx}(:,2); % y coefficients
        pz = C{idx}(:,3); % z coefficients
        pp = C{idx}(:,4); % psi coefficients
        pos = [polyval(px, q);
               polyval(py, q);
               polyval(pz, q)];
        yaw = polyval(pp, q);
        % Velocity
        px = polyder(px); % x coefficients
        py = polyder(py); % y coefficients
        pz = polyder(pz); % z coefficients
        pp = polyder(pp); % psi coefficients
        vel = [polyval(px, q);
               polyval(py, q);
               polyval(pz, q)];
        yawdot = polyval(pp, q);
        % Acceleration
        px = polyder(px); % x coefficients
        py = polyder(py); % y coefficients
        pz = polyder(pz); % z coefficients
        acc = [polyval(px, q);
               polyval(py, q);
               polyval(pz, q)];
    end
    
    % Get desired state
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
end

end