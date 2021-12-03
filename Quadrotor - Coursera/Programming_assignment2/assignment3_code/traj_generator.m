function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 N C alpha_x alpha_y alpha_z
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    [C,N]=size(waypoints);
    N = N-1;
    
    % Declaration
    A = zeros(8*N,8*N);
    
    beta_x = zeros(8*N,1);
    beta_y = zeros(8*N,1);
    beta_z = zeros(8*N,1);    
    % case i
    p_i    = [1 zeros(1,7)];
    p_i_DS = ones(1,8);
    for i=1:N
        A(i,8*(i-1)+1:8*i) = p_i;
        A(i+N,8*(i-1)+1:8*i) = p_i_DS;
    end
    
    for i=1:N
        beta_x(i,1) = waypoints(1,i);
        beta_y(i,1) = waypoints(2,i);
        beta_z(i,1) = waypoints(3,i);
    end
    
    for i=N+1:2*N
        beta_x(i,1) = waypoints(1,i+1-N);
        beta_y(i,1) = waypoints(2,i+1-N);
        beta_z(i,1) = waypoints(3,i+1-N);
    end
    
    % case ii
    dp_i   = [0 1 zeros(1,6)];
    ddp_i  = [zeros(1,2) 2 zeros(1,5)];
    dddp_i = [zeros(1,3) 6 zeros(1,4)];
    d4p_i = [zeros(1,4) 24 zeros(1,3)];
    d5p_i = [zeros(1,5) 120 zeros(1,2)];
    d6p_i = [zeros(1,6) 720 0];
    
    A(2*N+1,1:8) = dp_i;
    A(2*N+2,1:8) = ddp_i;
    A(2*N+3,1:8) = dddp_i;
    A(2*N+4,8*N-7:8*N) = [0 1 2 3 4 5 6 7];
    A(2*N+5,8*N-7:8*N) = [0 0 4 6 12 20 30 42];
    A(2*N+6,8*N-7:8*N) = [0 0 0 6 24 60 120 210];
    
    % case iii
    dp_i_DS = [0 1 2 3 4 5 6 7];
    ddp_i_DS = [0 0 2 6 12 20 30 42];
    dddp_i_DS = [0 0 0 6 24 60 120 210];
    d4p_i_DS = [0 0 0 0 24 120 360 840];
    d5p_i_DS = [0 0 0 0 0 120 720 2520];
    d6p_i_DS = [0 0 0 0 0 0 720 5040];
    
    for i=1:N-1
        A(2*N+i+6,8*(i-1)+1:8*i) = dp_i_DS;
        A(2*N+i+6,8*i+1:8*(i+1)) = -dp_i;
        
        A(3*N+i+5,8*(i-1)+1:8*i) = ddp_i_DS;
        A(3*N+i+5,8*i+1:8*(i+1)) = -ddp_i;
        
        A(4*N+i+4,8*(i-1)+1:8*i) = dddp_i_DS;
        A(4*N+i+4,8*i+1:8*(i+1)) = -dddp_i;
        
        A(5*N+i+3,8*(i-1)+1:8*i) = d4p_i_DS;
        A(5*N+i+3,8*i+1:8*(i+1)) = -d4p_i;
        
        A(6*N+i+2,8*(i-1)+1:8*i) = d5p_i_DS;
        A(6*N+i+2,8*i+1:8*(i+1)) = -d5p_i;
        
        A(7*N+i+1,8*(i-1)+1:8*i) = d6p_i_DS;
        A(7*N+i+1,8*i+1:8*(i+1)) = -d6p_i;
    end
    
    alpha_x = inv(A)*beta_x;
    alpha_y = inv(A)*beta_y;
    alpha_z = inv(A)*beta_z;
    
else
    if(t > traj_time(end))
        t = traj_time(end);
        
    end
    t_index = find(traj_time >= t,1);
    
    if(t_index>1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        vx = 0;
        vy = 0;
        vz = 0;
        
        ax = 0;
        ay = 0;
        az = 0;    
        
    else
        t_ = t/d0(t_index-1);
        i = t_index-1;
        
        px = alpha_x(8*(i-1)+1:8*i,1)'*[1;t_;...
            t_^2;t_^3;t_^4;...
            t_^5;t_^6;t_^7];
        vx = alpha_x(8*(i-1)+1:8*i,1)'*[0;1;...
            2*t_;3*t_^2;...
            4*t_^3;5*t_^4;...
            6*t_^5;7*t_^6];
        ax = alpha_x(8*(i-1)+1:8*i,1)'*[0;0;...
            2;6*t_;...
            12*t_^2;20*t_^3;...
            30*t_^4;42*t_^5];
        
        py = alpha_y(8*(i-1)+1:8*i,1)'*[1;t_;...
            t_^2;t_^3;t_^4;...
            t_^5;t_^6;t_^7];
        vy = alpha_y(8*(i-1)+1:8*i,1)'*[0;1;...
            2*t_;3*t_^2;...
            4*t_^3;5*t_^4;...
            6*t_^5;7*t_^6];
        ay = alpha_y(8*(i-1)+1:8*i,1)'*[0;0;...
            2;6*t_;...
            12*t_^2;20*t_^3;...
            30*t_^4;42*t_^5];
        
        pz = alpha_z(8*(i-1)+1:8*i,1)'*[1;t_;...
            t_^2;t_^3;t_^4;...
            t_^5;t_^6;t_^7];
        vz = alpha_z(8*(i-1)+1:8*i,1)'*[0;1;...
            2*t_;3*t_^2;...
            4*t_^3;5*t_^4;...
            6*t_^5;7*t_^6];
        az = alpha_z(8*(i-1)+1:8*i,1)'*[0;0;...
            2;6*t_;...
            12*t_^2;20*t_^3;...
            30*t_^4;42*t_^5];

        
        
        desired_state.pos = [px;py;pz];
        
    end
 
    desired_state.vel = [vx;vy;vz];
    desired_state.acc = [ax;ay;az];
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end



%% Fill in your code here 

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

