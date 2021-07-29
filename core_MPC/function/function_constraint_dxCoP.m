function [A,b]=function_constraint_dxCoP(CoP_state_preview,dxCoP_max)


%% COM accel dCOP < value
    A_dCOP=CoP_state_preview.Pu_dz_mean;
    b_dCOP=dxCoP_max-CoP_state_preview.f_dz_mean(:,1);  

A=A_dCOP;
b=b_dCOP;