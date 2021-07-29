%% Initialize MPC storage of MPC iteration from QP result
physical_model_storage=classdef_physical_model;

% COM along x axis 
physical_model_storage.xc(1)=robot.xcom_0(1);
physical_model_storage.xdc(1)=robot.xcom_0(2);
physical_model_storage.xddc(1)=robot.xcom_0(3);
  
% COM along y axis
physical_model_storage.yc(1)=robot.ycom_0(1);
physical_model_storage.ydc(1)=robot.ycom_0(2);
physical_model_storage.yddc(1)=robot.ycom_0(3);

% COM along z axis 
physical_model_storage.zc(1)=robot.zcom_0(1);
physical_model_storage.zdc(1)=robot.zcom_0(2);
physical_model_storage.zddc(1)=robot.zcom_0(3); 
 
% foot step
if size(experiment.phase_type,1)==1  %&& experiment.phase_type == "start" 
        physical_model_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        physical_model_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        physical_model_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
else
    if experiment.phase_type(2)=='r'
        physical_model_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        physical_model_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        physical_model_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
    elseif experiment.phase_type(2)=='l'
        physical_model_storage.xstep=[robot.xstep_r_0;robot.xstep_l_0];
        physical_model_storage.ystep=[robot.ystep_r_0;robot.ystep_l_0];
        physical_model_storage.zstep=[experiment.zstep_r_0;experiment.zstep_l_0];
    else 
        physical_model_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        physical_model_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        physical_model_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
    end
end


    physical_model_storage.xzmp=0;
    physical_model_storage.yzmp=0;
    physical_model_storage.zzmp=0;