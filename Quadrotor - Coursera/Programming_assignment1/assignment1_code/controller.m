function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

K_p = 20;
K_v = 3;

err_p = s_des(1)-s(1);
err_v = s_des(2)-s(2);

u = params.mass*params.gravity+K_p*err_p+K_v*err_v;


end

