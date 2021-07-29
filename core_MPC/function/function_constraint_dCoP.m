function [A,b]=function_constraint_dCoP(CoP_state_preview,Step_state_preview,dCoP_max)
 
    dCOPx2=classdef_CostPart(CoP_state_preview.Pu_dz_mean,...
    [],...
    CoP_state_preview.f_dz_mean(:,1),...
    CoP_state_preview.f_dz_mean(:,1)*0 ...
    );
  
    zA = dCOPx2.H.*0; 
    xA = dCOPx2.H;
    if ~isempty(Step_state_preview.Pu_step)
    xA = [dCOPx2.H,zeros(size(Step_state_preview.Pu_step,2))];
    end
    dCOPy2=classdef_CostPart(CoP_state_preview.Pu_dz_mean,...
    [],...
    CoP_state_preview.f_dz_mean(:,2),...
    CoP_state_preview.f_dz_mean(:,2)*0 ...
    );

    yA = dCOPy2.H;
    if ~isempty(Step_state_preview.Pu_step)
    yA = [dCOPy2.H,zeros(size(Step_state_preview.Pu_step,2))];
    end
%% COP vel dCOP < value
    
    %% Concatenate Cost function along all axis
    A_dCOP=[xA,yA,zA];  
    b_dCOP=-dCOPx2.f-dCOPx2.f+dCoP_max;

A=A_dCOP; 
b=b_dCOP;