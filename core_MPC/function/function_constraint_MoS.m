function [A,b]=function_constraint_MoS(MPC_inputs,COM_state_preview)
 
AP_limit=(MPC_inputs.xstep(2)+MPC_inputs.fronttoankle);
ML_limit=(MPC_inputs.ystep(2)-MPC_inputs.inttoankle);

Pu_xcom = COM_state_preview.Pu_c + (1/MPC_inputs.omega_temp)*COM_state_preview.Pu_dc;
f_xcom  = COM_state_preview.f_c(:,1) + (1/MPC_inputs.omega_temp)*COM_state_preview.f_dc(:,1);
f_ycom  = COM_state_preview.f_c(:,2) + (1/MPC_inputs.omega_temp)*COM_state_preview.f_dc(:,2);


MoS_AP=0.20;
MoS_ML=-0.035;
MoS_ML=-0.0; 
ML_limit=MPC_inputs.ystep(2)-MPC_inputs.inttoankle;

% A=-Pu_xcom.*diag(MPC_inputs.MoS_sampling);
% bx=diag(MPC_inputs.MoS_sampling)*(-AP_limit+f_xcom+MoS_AP);
% by=diag(MPC_inputs.MoS_sampling)*(-ML_limit+f_ycom+MoS_ML);

% AP_limit-MoS_AP
% ML_limit-MoS_ML
% bx=diag(MPC_inputs.MoS_sampling)*(f_xcom);
% by=diag(MPC_inputs.MoS_sampling)*(f_ycom);

% A=Pu_xcom.*diag(MPC_inputs.MoS_sampling);
% bx=diag(MPC_inputs.MoS_sampling)*(f_xcom);
% by=diag(MPC_inputs.MoS_sampling)*(-f_ycom+ML_limit+MoS_ML);

MoS_ML=0.05; 
A=-Pu_xcom.*diag(MPC_inputs.MoS_sampling);
bx=diag(MPC_inputs.MoS_sampling)*(-f_xcom+0.09);
% by=diag(MPC_inputs.MoS_sampling)*(f_ycom+MoS_ML);
by=diag(MPC_inputs.MoS_sampling)*(f_ycom-0.03);


b = [bx; by];





