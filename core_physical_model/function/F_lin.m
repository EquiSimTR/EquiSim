function dx = F_lin(~,x,L)
% Derivative function for a linear pendulum model.
%
% States:
%   x(1):   theta
%   x(2):   d theta/dt

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

dx(1,1) = x(2);
dx(2,1) = -g/L*x(1);