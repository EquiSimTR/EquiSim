%% Initialize Model storage  
sensor_dynamics_storage=classdef_sensor_dynamics;
 
% COM along x axis 
sensor_dynamics_storage.xc(1)=robot.xcom_0(1);
sensor_dynamics_storage.xdc(1)=robot.xcom_0(2);
sensor_dynamics_storage.xddc(1)=robot.xcom_0(3);

% COM along y axis 
sensor_dynamics_storage.yc(1)=robot.ycom_0(1);
sensor_dynamics_storage.ydc(1)=robot.ycom_0(2);
sensor_dynamics_storage.yddc(1)=robot.ycom_0(3);

% COM along z axis  
sensor_dynamics_storage.zc(1)=robot.zcom_0(1);
sensor_dynamics_storage.zdc(1)=robot.zcom_0(2);
sensor_dynamics_storage.zddc(1)=robot.zcom_0(3);

% foot step
if size(experiment.phase_type,1)==1  %&& experiment.phase_type == "start" 
        sensor_dynamics_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        sensor_dynamics_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        sensor_dynamics_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
else
    if experiment.phase_type(2)=='r'
        sensor_dynamics_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        sensor_dynamics_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        sensor_dynamics_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
    elseif experiment.phase_type(2)=='l'
        sensor_dynamics_storage.xstep=[robot.xstep_r_0;robot.xstep_l_0];
        sensor_dynamics_storage.ystep=[robot.ystep_r_0;robot.ystep_l_0];
        sensor_dynamics_storage.zstep=[experiment.zstep_r_0;experiment.zstep_l_0];
    else 
        sensor_dynamics_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
        sensor_dynamics_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
        sensor_dynamics_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
    end
end