function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE

% Position error and Velocity error
e_p = des_state.pos - state.pos;
e_v = des_state.vel - state.vel;

% Gain matrices
K_p = diag([30 30]);
K_v = diag([3 3]);

K_p_phi = 1;
K_v_phi = 0.01;

u1 = params.mass * (params.gravity + des_state.acc(2)...
    + K_p(2,2)*e_p(2) + K_v(2,2)*e_v(2));

ddy_dtt_c = des_state.acc(1) + K_p(1,1)*e_p(1)...
    +K_v(1,1)*e_v(1);

phi_c = - 1/params.gravity * ddy_dtt_c;
dphi_dt_c = -1/params.gravity*(K_v(1,1)*(des_state.acc(1))...
    + K_p(1,1)*(des_state.vel(1) - state.vel(1)));

u2 =K_p_phi*(phi_c - state.rot) + K_v_phi*(dphi_dt_c - state.omega);
end

