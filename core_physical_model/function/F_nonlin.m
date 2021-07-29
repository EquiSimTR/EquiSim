function dx = F_nonlin(~,x,L)
% Derivative function for a nonlinear pendulum model.
%
% States:
%   x(1):   theta
%   x(2):   d theta/dt

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

dx(1,1) = x(2);
dx(2,1) = -g/L*sin(x(1));