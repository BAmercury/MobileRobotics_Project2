function [DT, C] = computeCoefficients(map, path, params)
% COMPUTECOEFFICIENTS compute coefficients of trajectory
%
% INPUTS:
% map    - BinaryOccupancyGrid3D, configuration space representation
% path   - 2 x m matrix, waypoints that the robot must pass through (each
%          column is a single way point)
% params - robot parameters, the output of the utils/crazyflie function
%
% OUTPUTS:
% DT     - 1 x m-1, vector of the time spent in each segment, ex. DT(3) is
%          the time it takes to go on segment 3, which connects waypoint 3 
%          to waypoint 4
% C      - 1 x m-1 cell array of 2*n x 4 matrices, where n is the order of
%          the system that you choose.  For example, C{3} is a 2*n x 4 
%          matrix that contains the coefficients of the 3rd segment, 
%          which connects waypoints 3 and 4.  The first column of C{k} has
%          the coefficients for the x coordinates, the second column for y,
%          the third column for z, and the fourth column for psi.  The
%          first row has the coefficients for the 2n-1 term and the last
%          row has the coefficients for the 0th order term.

m = size(path,2); % number of waypoints

DT = zeros(1, m-1); % dummy values




C = cell(1, m-1); % initialize cell array
for i = 1:m-1
    head = path(:,i);
    base = path(:,i+1);
    x = ((head(1) - base(1)^2));
    y = ((head(2) - base(2)^2));
    z = ((head(3) - base(3)^2));
    dist = abs(sqrt(x+y+z));
    DT(i+1) = dist/params.avg_vel;
    T = DT(i+1);
    
    states = [head(1) head(2) head(3) 0; base(1) base(2) base(3) 0; 0 0 0 0; 0 0 0 0];
    t = [0 0 0 1;(T^3) (T^2) T 1; 0 0 1 0; (3*(T^2)) (2*T) 1 0]
    inv_t = inv(t);
    
    c = inv_t * states;
   
    
    C{i+1} = c; % dummy values
    % This creates a 0th order polynomial in x, y, z, and yaw with a
    % constant value equal to [start_x, start_y, start_z, 0]
end
