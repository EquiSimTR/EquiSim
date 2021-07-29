% human description
%% default CoM height

h_com=0.780678;
% h_com=0.560*1.61;

%% COM height limits to the floor
%with respect to h_com
h_com_max=+0.05;
h_com_min=-0.25;

%% Initial standing state default
%% release angle
release_angle = 0;
switch release_angle
    case 0
        theta=0; % (°) degrees 
    case 1
        theta=13.5; % (°) degrees; longueur de pas 0.72 temps 0.50 
    case 2
        theta=17.3; % (°) degrees; longueur de pas 0.82 temps 0.49 
    case 3
        theta=20.6; % (°) degrees; longueur de pas 0.88 temps 0.46 
    case 4
        theta=22.7; % (°) degrees; longueur de pas 1.02 temps 0.47 
    case 5
        theta=26.7; % (°) degrees; longueur de pas 1.10 temps 0.46 
    case 6
        theta=30.8; % (°) degrees; longueur de pas 1.18 temps 0.45   
end
D2R=pi/180; % degrees to radians
theta = theta * D2R; % convert theta from degrees to radians
g=9.81; %m.s-1
omega_temp=sqrt(g/(h_com*cos(theta)));
zeta_temp=1/omega_temp^2;
 
%% Initial standing state default
xcom_0=[h_com*sin(theta);0;((h_com*sin(theta))/zeta_temp)*1]; %5.8479 
% xcom_0=[h_com*sin(theta);0;-10]; 
ycom_0=[0;0;0];
zcom_0=[h_com*cos(theta);0;0];

% xstep_r_0=0;
% ystep_r_0=-0.0815817;
% 
% xstep_l_0=0;
% ystep_l_0=0.0815817;

xstep_r_0=0;
ystep_r_0=-0.12;

xstep_l_0=0;
ystep_l_0=0.12;

%% Foot limits
backtoankle=0.05; %from back to ankle of foot
fronttoankle=0.2; %from  front to ankle of foot
exttoankle=0.05; %from exterior to ankle of foot
inttoankle=0.025; %from interior to ankle of foot

sole_margin=0.002;

%% Foot step placement limits
xankmax=0.8;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=2*inttoankle+0.0552;%0.15;%width min between ankles
% yankmin=2*inttoankle+0.0399;%0.15;%width min between ankles
% yankmin=0.1769
yankmax=2*inttoankle+0.4;%width max between ankles
% yankmax=2*inttoankle+0.0552;%width max between ankles
