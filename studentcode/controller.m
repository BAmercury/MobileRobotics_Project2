function [F, M] = controller(qd, t, params)
% CONTROLLER quadrotor controller
%
% INPUTS:
% qd     - 1 x n cell, qd structure, contains current and desired state
% t      - 1 x 1, time
% params - struct, output from crazyflie() and whatever parameters you want to pass in
%
% OUTPUTS:
% F      - 1 x 1, thrust (u1)
% M      - 3 x 1, moments (u2, u3, u4)

% Convert qd to state
[s] = qdToState(qd);

% Get desired state
pos_des = qd.pos_des;
vel_des = qd.vel_des;
acc_des = qd.acc_des;
psi_des = qd.yaw_des;
wx_des = 0;
wy_des = 0;
wz_des = qd.yawdot_des;

% Assign current states
x    = s(1); % Position
y    = s(2);
z    = s(3);
xdot = s(4); % Linear velocity
ydot = s(5);
zdot = s(6);
qW   = s(7); % Quaternion
qX   = s(8);
qY   = s(9);
qZ   = s(10);
wx   = s(11); % Angular velocity
wy   = s(12);
wz   = s(13);
Rot  = QuatToRot([qW; qX; qY; qZ]);
[phi, theta, psi] = RotToRPY_ZXY(Rot);

% Extract parameters of the robot
g = params.grav;
m = params.mass;
I = params.I;
L = params.arm_length;

%% YOUR CODE STARTS HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute desired linear accelerations, used for altitude controller
gain_px = 1;
gain_py = 1;
gain_dx = 2;
gain_dy = 2;


xa = acc_des(1) + (gain_dx*(vel_des(1)-xdot)) + (gain_px*(pos_des(1)-x));
ya = acc_des(2) + (gain_dy*(vel_des(2)-ydot)) + (gain_py*(pos_des(2)-y));



% Compute desired roll and pitch angle based on desired acceleration and
% yaw angle, used for altitude controller
roll_des = (1/g)*(xa*sin(phi))-(ya*cos(phi));
pitch_des = (1/g)*(xa*cos(phi))+(ya*sin(phi));

% Compute force (u1), position controller
gain_pz = 1;
gain_dz = 2;

u1 = m*(g + acc_des(3) + (gain_dz*(vel_des(3)-zdot))+ (gain_pz*(pos_des(3)-z)) );

gain_pphi = 20;
gain_ptheta = 20;
gain_ppsi = 0;

gain_dphi = 8;
gain_dtheta = 8;
gain_dpsi = 0;



u2 = I(1) * ( (gain_dphi*(0-wx)) + (gain_pphi*(roll_des - phi)));
u3 = I(1) * ( (gain_dtheta*(0-wy)) + (gain_ptheta*(pitch_des - theta)));
u4 = I(9) * ( (gain_ppsi*(wz_des-wz)) + (gain_ppsi*(psi_des - psi)));
%% YOUR CODE ENDS HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set outputs
F = u1;
M = [u2; u3; u4];

disp(u2);

end
