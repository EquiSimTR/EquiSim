function [A,b]=function_constraint_dyCoP(CoP_state_preview,dyCoP_max)


%% COM accel dCOP < value
    A_dCOP=CoP_state_preview.Pu_dz_mean;
    b_dCOP=dyCoP_max-CoP_state_preview.f_dz_mean(:,2);  

A=A_dCOP;
b=b_dCOP;