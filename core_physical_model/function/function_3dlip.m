function segma = function_3dlip(c,dc,z,p,t,g,Ts)
%FUNCTION_LIP Summary of this function goes here
%   Detailed explanation goes here
omega = sqrt(z/g);
            
A = [0 1; omega^2 0];
B = [0; -omega^2];

segma0 = [c dc]';
segma = zeros(3,1);
segma(1:2)= [c dc]';
% Repeat application of Euler method sampled at Ts/M.
% sys = ss(A,B,[],[]);
% sysd = c2d(sys,Ts);
% 
% M = 10;
% delta = Ts/M; 
% 
% for dt=1:M
%     segma(1:2) = segma(1:2) + delta*(sysd.A * segma(1:2)+ sysd.B*p);
% end

% for dt=1:M
%     segma(1:2) = segma(1:2) + delta*([cosh(omega*t) omega^-1*sinh(omega*t);...
%     omega*sinh(omega*t) cosh(omega*t)] * segma(1:2)+...
%     [1-cosh(omega*t) ...
%     -omega*sinh(omega*t)]'*p);
% end
 
segma(1:2) = [cosh(omega*t) omega^-1*sinh(omega*t);...
    omega*sinh(omega*t) cosh(omega*t)] * segma0+...
    [1-cosh(omega*t) ...
    -omega*sinh(omega*t)]'*p;
 
segma(3) = omega^2*(c-p);
 
end

