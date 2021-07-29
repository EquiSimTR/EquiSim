

clear all 
path(pathdef); %clear the pathdef of include library
clc

% addpath_core_test = ["core_test_all_axis/function/ core_test_all_axis/classdef/"];
% addpath addpath_core_test
% currentFolder = pwd;
% cc= fullfile(currentFolder,'core_test_all_axis/function')

%addpath script/ 
addpath core_physical_model/function/ core_physical_model/classdef/ core_physical_model/script/
addpath core_test_all_axis/function/ core_test_all_axis/classdef/  
addpath core_MPC/classdef/ core_MPC/classdef/linear_trajectories/ core_MPC/function/ core_MPC/script/

%%
robot_type='human';
%hrp4
%human

phase_duration_type='phase_duration_01';

walking_type=4;
% 1 : walking flat
% 2 : walking airbus stairs
% 3 : walking flat quick
% 4 : walking flat fixed foot step positions
% 5 : walking airbus stairs fixed foot step positions

% controller='vrep';
% vrep
% rviz

cop_ref_type='ankle_center';
%'ankle_center' : polyhedron centered on the ankle
%'foot_center' : polyhedron centered on the middle of the foot  

polyhedron_position='waist_center';
%'ankle_center' : polyhedron centered on the ankle
%'foot_center' : polyhedron centered on the middle of the foot
%'waist_center' : polyhedron centered on the middle of the waist   

kinematic_limit='hexagonTranslation';
%'' : very simple polyhedron
%'hexagon' : hexagon kinematic limits
%'hexagonTranslation' : hexagon kinematic limits with translation

COM_form='comPolynomial';
%'comPolynomial' : COM with piece-wise jerk
%'comExponential' : ZMP with piece-wise velocity
%'comPolyExpo' : COM with polynomial of exponential

%b = both feet; r = right foot; l = left foot
nb_foot_step=0;
attime=1; 
firstSS='r';

save_figure=false;

movie_record=false;

%% Constant
run('core_test_all_axis/script/script_constant.m')

%% Init storage QP result
run('core_test_all_axis/script/script_init_storage_qp_result.m')

%% Init storage Physical model states
run('core_physical_model/script/script_init_storage_physical_model.m')

%% Init storage sensor dynamics
run('core_physical_model/script/script_init_storage_sensor_dynamics.m')
 
%% movie record
if movie_record  
    v_COM = VideoWriter('COM_MPC.avi');
    v_COM.Quality = 95;
    v_COM.FrameRate=10;
    open(v_COM);
    
    v_CoP = VideoWriter('CoP_MPC.avi');
    v_CoP.Quality = 95;
    v_CoP.FrameRate=10;
    open(v_CoP);
end
%% Optimization problem QP
% Sampling update
tic
k=0;
% for k=1:experiment.phase_duration_iteration_cumul(end)
while k<=experiment.phase_duration_iteration_cumul(end)
    k=k+1;
    if experiment.phase_duration_sampling_cumul(k)>=attime && experiment.phase_duration_sampling_cumul(k)<attime+experiment.phase_duration_sampling(k)
            experiment.classdef_create_experiment_new(phase_duration_type,...
            nb_foot_step,firstSS,...
            kinematic_limit,robot,polyhedron_position,...
            optimWeight_filename,...
            walking_type,k); 
    end
    %% creation of inputs of a MPC iteration 

%     MPC_inputs=classdef_MPC_problem_inputs;
    MPC_inputs=function_fill_MPC_inputs(robot,experiment,MPC_outputs_storage,...
        COM_form,kinematic_limit,cop_ref_type,firstSS,...
        k);
%     run('script/script_problem_iteration_creation.m')
    
    %% linear MPC iteration
    MPC_outputs=classdef_MPC_problem_outputs;
    
    problem_building_filename=[...
        "script_initialize_from_current_state.m";...% Initialization from last robot state
        "script_update_cost.m";...% Cost
        "script_cons_ineq.m";...% Constraint Equalities
        "script_cons_eq.m"...% Constraint Inequalities
        ];
    
    MPC_outputs.MPC_iteration(MPC_inputs,problem_building_filename);
        
    %% Store MPC results
    MPC_outputs_storage.add_storage(MPC_outputs);
    
    %% linear MPC iteration
    physical_model=classdef_physical_model;
    physical_model.physical_model_iteration_F_lin(experiment,MPC_outputs,physical_model_storage,k);
     
    %% Store MPC results
    physical_model_storage.add_storage(physical_model); 
%     physical_model_storage.add_storage_sensors(physical_model);
 
    sensor_dynamics=classdef_sensor_dynamics;
%     sensor_dynamics.sensor_dynamics_iteration(physical_model);
	neural_time_delay = 0; % ms
    sensor_dynamics.sensor_dynamics_iteration(experiment,physical_model_storage,neural_time_delay,k);
     
    sensor_dynamics_storage.add_storage(sensor_dynamics);
     
        %% Plot actual states  
    function_plot_each_iteration(MPC_outputs_storage, physical_model_storage, ... 
                                 sensor_dynamics_storage,experiment,...
                                 'time'                 ,'on', ...
                                 'MPC_CoM'              ,'on', ...
                                 'PHY_MDL_CoM'          ,'off', ...
                                 'SEN_MDL_CoM'          ,'off', ...
                                 'CoP'                  ,'on', ...
                                 'x'                    ,'on', ...
                                 'y'                    ,'on', ...
                                 'z'                    ,'on', ...
                                 'drawing footsteps','off',k);
    %%
%     run('script/script_display_online.m')
    run('core_test_all_axis/script/script_movie_record.m')
end
toc
 
if movie_record
    close(v_COM)
    close(v_CoP)
end

%% Results ZMP
run('core_test_all_axis/script/script_zmp.m')

%% foot traj in the air
% hstep_move=0.05;
hstep_move=0.2;
run('core_test_all_axis/script/script_foot_traj_air.m')
% run('script/script_foot_traj_air_stairs.m')

%% discretization trajectories
run('core_test_all_axis/script/script_traj_discretization.m')

%% 
dt_type_phase_=any(phase_type_sampling_enlarge=='l',2)*1+any(phase_type_sampling_enlarge=='r',2)*2;
dt_type_phase_=[0;dt_type_phase_];

%% Plot results
run('core_test_all_axis/script/script_plot_results.m')
return;    
%% write txt
if false  
    run('core_test_all_axis/script/script_write_txt.m')
end
%%
if save_figure
    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D'])
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta'])
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal'])
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal'])
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView'])

    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D']...
        ,'png')
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta']...
        ,'png')
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal']...
        ,'png')
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal' ]...
        ,'png')
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView' ]...
        ,'png')
end

return
figHandles = findall(0,'Type','figure'); 
currentFolder = pwd;
% Create filename 
% fn = tempname();  %in this example, we'll save to a temp directory.
% Specify some particular, specific folder:
 
fn = fullfile(currentFolder, 'results', 'figure');
% Save first figure
 export_fig([fn,num2str(figHandles(1).Number)], '-pdf', figHandles(1))
 export_fig([fn,num2str(figHandles(1).Number)], '-eps', figHandles(1))
 
% Loop through figures 2:end
 for i = 2:numel(figHandles) 
     export_fig([fn,num2str(figHandles(i).Number)], '-pdf', figHandles(i), '-append')
     export_fig([fn,num2str(figHandles(i).Number)], '-eps', figHandles(i), '-append')
 end
  
doc_path = fullfile(currentFolder, 'results'); 
winopen(doc_path) %assuming fn is the full path the document
 
 
% figures to keep
figs2keep = [133]; 
% Uncomment the following to 
% include ALL windows, including those with hidden handles (e.g. GUIs)
% all_figs = findall(0, 'type', 'figure'); 
all_figs = findobj(0, 'type', 'figure');
delete(setdiff(all_figs, figs2keep));