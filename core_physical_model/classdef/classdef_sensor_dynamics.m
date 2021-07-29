classdef classdef_sensor_dynamics<handle
    %CLASS_PHYSICAL_MODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        xc %COM end position of the next iteration along x-axis
        xdc %COM end velocity of the next iteration along x-axis
        xddc %COM end acceleration of the next iteration along x-axis
        
        yc %COM end position of the next iteration along y-axis
        ydc %COM end velocity of the next iteration along y-axis
        yddc %COM end acceleration of the next iteration along y-axis

        zc %COM end position of the next iteration along z-axis
        zdc %COM end velocity of the next iteration along z-axis
        zddc %COM end acceleration of the next iteration along z-axis 

        xstep %foot step position along x-axis
        ystep %foot step position along y-axis
        zstep %foot step position along z-axis (not an optimization variable)

        xzmp %zmp position along x-axis
        yzmp %zmp position along y-axis
        zzmp %zmp position along z-axis (not an optimization variable)
        
%         duration
    end
    
    methods
        function obj = classdef_sensor_dynamics()
            %Empty constructor
            %
            %CLASSDEF_PHYSICAL_MODEL_OUTPUTS/class_physical_model is a function.
            %    obj = class_physical_model()
        end
        function obj=add_storage(obj,sensor_dynamics)
            %Stores the properties value of the next iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/add_storage is a function.
            %    obj = add_storage(obj, sensor_dynamics)
            %
            %Stores the properties value of 'sensor_dynamics'
            %'sensor_dynamics' is a classdef_MPC_problem_outputs object
            %
            %all properties, but QP_result_all, are stored as vector
            %QP_result_all is stored as array of cells accessible with {}
            %instead of ()
             
            obj.xc(end+1,:)=sensor_dynamics.xc;
            obj.xdc(end+1,:)=sensor_dynamics.xdc;
            obj.xddc(end+1,:)=sensor_dynamics.xddc;
 
            obj.yc(end+1,:)=sensor_dynamics.yc;
            obj.ydc(end+1,:)=sensor_dynamics.ydc;
            obj.yddc(end+1,:)=sensor_dynamics.yddc;
 
            obj.zc(end+1,:)=sensor_dynamics.zc;
            obj.zdc(end+1,:)=sensor_dynamics.zdc;
            obj.zddc(end+1,:)=sensor_dynamics.zddc;
            
            if ~isempty(sensor_dynamics.xstep)
                obj.xstep(end+1,:)=sensor_dynamics.xstep;
                obj.ystep(end+1,:)=sensor_dynamics.ystep;
                obj.zstep(end+1,:)=sensor_dynamics.zstep;
            end  
            
%             obj.xzmp(end+1,:)=sensor_dynamics.xzmp;
%             obj.yzmp(end+1,:)=sensor_dynamics.yzmp;
%             obj.zzmp(end+1,:)=sensor_dynamics.zzmp;
        end
         
        %%  
        function obj = sensor_dynamics_iteration(obj,experiment,sensor_dynamics_storage,neural_time_delay,k)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here 
            sampling_time = experiment.phase_duration_sampling(k);
            n = function_delay(sampling_time,neural_time_delay,k);
            
            obj.xc=sensor_dynamics_storage.xc(n)+0.005*(-1 + 2*rand);
            obj.xdc=sensor_dynamics_storage.xdc(n)+0.0*(-1 + 2*rand); 
            obj.xddc=sensor_dynamics_storage.xddc(n)+0.0*(-1 + 2*rand);
             
            obj.yc=sensor_dynamics_storage.yc(n)+0.005*(-1 + 2*rand);
            obj.ydc=sensor_dynamics_storage.ydc(n)+0.0*(-1 + 2*rand);
            obj.yddc=sensor_dynamics_storage.yddc(n)+0.0*(-1 + 2*rand);
             
            obj.zc=sensor_dynamics_storage.zc(n)+0.005*(-1 + 2*rand);
            obj.zdc=sensor_dynamics_storage.zdc(n)+0.0*(-1 + 2*rand);
            obj.zddc=sensor_dynamics_storage.zddc(n)+0.0*(-1 + 2*rand);  
             
            if ~isempty(sensor_dynamics_storage.xstep)
                obj.xstep(end+1,:)=sensor_dynamics_storage.xstep(end);
                obj.ystep(end+1,:)=sensor_dynamics_storage.ystep(end);
                obj.zstep(end+1,:)=sensor_dynamics_storage.zstep(end);
            end 
             
        end
        %%        
        function obj = sensor_dynamics_iteration1(obj,physical_model)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
             
            obj.xc=physical_model.xc+0.00*(-1 + 2*rand);
            obj.xdc=physical_model.xdc+0.0*(-1 + 2*rand); 
            obj.xddc=physical_model.xddc+0.0*(-1 + 2*rand);
             
            obj.yc=physical_model.yc+0.00*(-1 + 2*rand);
            obj.ydc=physical_model.ydc+0.0*(-1 + 2*rand);
            obj.yddc=physical_model.yddc+0.0*(-1 + 2*rand);
             
            obj.zc=physical_model.zc+0.00*(-1 + 2*rand);
            obj.zdc=physical_model.zdc+0.0*(-1 + 2*rand);
            obj.zddc=physical_model.zddc+0.0*(-1 + 2*rand);  
             
            if ~isempty(physical_model.xstep)
                obj.xstep(end+1,:)=physical_model.xstep;
                obj.ystep(end+1,:)=physical_model.ystep;
                obj.zstep(end+1,:)=physical_model.zstep;
            end 
             
        end
    end
end

