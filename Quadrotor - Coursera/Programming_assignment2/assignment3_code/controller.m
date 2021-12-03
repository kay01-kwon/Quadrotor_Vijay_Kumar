function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
m = params.mass;
g = params.gravity;


% Position controller
% Gains
K_p1 = diag([300 300 300]);
K_v1 = diag([40 40 35]);

% Position and Velocity Error
e_p1 = des_state.pos-state.pos;
e_v1 = des_state.vel-state.vel;

% Acceleration command
accel_cmd = des_state.acc+K_p1*e_p1+K_v1*e_v1;

% Thrust
F = m*(g+accel_cmd(3));

% Attitude controller
% Desired euler angles
phi_des = 1/g*(accel_cmd(1)*sin(des_state.yaw)-accel_cmd(2)*cos(des_state.yaw));
theta_des = 1/g*(accel_cmd(1)*cos(des_state.yaw)+accel_cmd(2)*sin(des_state.yaw));
psi_des =des_state.yaw;

% Inertia of moment

M = diag([100 100 100]) * [phi_des-state.rot(1);theta_des-state.rot(2);psi_des-state.rot(3)]...
     +diag([2 2 2]) * [0-state.omega(1);0-state.omega(2);des_state.yawdot-state.omega(3)];


% phi = state.rot(1);
% theta = state.rot(2);
% psi = state.rot(3);
% R = [cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta) -cos(phi)*sin(psi) cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
%     cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta) cos(phi)*cos(psi) sin(psi)*sin(theta)-cos(theta)*sin(phi)*cos(psi);
%     -cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)];
% 
% R_des = [cos(psi_des)*cos(theta_des)-sin(phi_des)*sin(psi_des)*sin(theta_des) -cos(phi_des)*sin(psi_des) cos(psi_des)*sin(theta_des)+cos(theta_des)*sin(phi_des)*sin(psi_des);
%     cos(theta_des)*sin(psi_des)+cos(psi_des)*sin(phi_des)*sin(theta_des) cos(phi_des)*cos(psi_des) sin(psi_des)*sin(theta_des)-cos(theta_des)*sin(phi_des)*cos(psi_des);
%     -cos(phi_des)*sin(theta_des) sin(phi_des) cos(phi_des)*cos(theta_des)];
% 
% M = diag([0.01 0.01 0.01])*(R'*R_des)*[1;1;1];
% =================== Your code ends here ===================

end
