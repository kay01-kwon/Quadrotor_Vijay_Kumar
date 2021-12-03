clear all
close all
N = 4;
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';

A = zeros(8*N,8*N);
alpha_x = zeros(8*N,1);
alpha_y = zeros(8*N,1);
alpha_z = zeros(8*N,1);

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
A(2*N+4,8*N-7:8*N) = dp_i;
A(2*N+5,8*N-7:8*N) = ddp_i;
A(2*N+6,8*N-7:8*N) = dddp_i;

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


t_ = linspace(0,1,50);
t2_ = linspace(1,2,50);
for i =1:100
    if i<51
        px(i) = alpha_x(1:8,1)'*[1;t_(i);...
            t_(i)^2;t_(i)^3;t_(i)^4;...
            t_(i)^5;t_(i)^6;t_(i)^7];
        vx(i) = alpha_x(1:8,1)'*[0;1;...
            2*t_(i);3*t_(i)^2;...
            4*t_(i)^3;5*t_(i)^4;...
            6*t_(i)^5;7*t_(i)^6];
    else
        px(i) = alpha_x(9:16,1)'*[1;(t2_(i-50)-1);...
            (t2_(i-50)-1)^2;(t2_(i-50)-1)^3;(t2_(i-50)-1)^4;...
            (t2_(i-50)-1)^5;(t2_(i-50)-1)^6;(t2_(i-50)-1)^7];
        
        vx(i) = alpha_x(9:16,1)'*[0;1;...
            2*t2_(i-50);3*t2_(i-50)^2;...
            4*t2_(i-50)^3;5*t2_(i-50)^4;...
            6*t2_(i-50)^5;7*t2_(i-50)^6];
    end
end

figure(1)
plot([t_ t2_],px)

figure(2)
plot([t_ t2_],vx)
