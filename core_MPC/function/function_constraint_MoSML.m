function [A,b]=function_constraint_MoSML(MPC_inputs,COM_state_preview)


Pu_xcom = COM_state_preview.Pu_c + (1/MPC_inputs.omega_temp)*COM_state_preview.Pu_dc;
f_xcom  = COM_state_preview.f_c(:,2) + (1/MPC_inputs.omega_temp)*COM_state_preview.f_dc(:,2);

MoS_ML = MPC_inputs.ystep(2) - MPC_inputs.inttoankle; 
 
ML = -0.045;
A=Pu_xcom*diag(MPC_inputs.MoS_sampling);
% b=diag(MPC_inputs.MoS_sampling)*(-f_xcom+MoS_ML+ML);
b=diag(MPC_inputs.MoS_sampling)*(-f_xcom);