%as Camille
w1=10^-5; %jerk -7
w2=0*10^-2; %com vel ref
w3=10^+0; %zmp wth zeta mean close to step
w4=0*10^-2; %com height

w5=0*10^-1; %zmp acceleration aka COM acceleration
w6=0*10^-2; %zmp vel between segment

w7=10^0; %step position

OptimCostWeight=[w1 w2 w3 w4 w5 w6 w7];
clear w1 w2 w3 w4 w5 w6 w7