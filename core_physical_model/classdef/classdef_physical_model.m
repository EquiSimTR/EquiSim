classdef classdef_physical_model<handle
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
        function obj = classdef_physical_model()
            %Empty constructor
            %
            %CLASSDEF_PHYSICAL_MODEL_OUTPUTS/class_physical_model is a function.
            %    obj = class_physical_model()
        end
        function obj=add_storage(obj,outputs)
            %Stores the properties value of the next iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/add_storage is a function.
            %    obj = add_storage(obj, outputs)
            %
            %Stores the properties value of 'outputs'
            %'outputs' is a classdef_MPC_problem_outputs object
            %
            %all properties, but QP_result_all, are stored as vector
            %QP_result_all is stored as array of cells accessible with {}
            %instead of ()
             
            obj.xc(end+1,:)=outputs.xc;
            obj.xdc(end+1,:)=outputs.xdc;
            obj.xddc(end+1,:)=outputs.xddc;
 
            obj.yc(end+1,:)=outputs.yc;
            obj.ydc(end+1,:)=outputs.ydc;
            obj.yddc(end+1,:)=outputs.yddc;
 
            obj.zc(end+1,:)=outputs.zc;
            obj.zdc(end+1,:)=outputs.zdc;
            obj.zddc(end+1,:)=outputs.zddc;
            
            if ~isempty(outputs.xstep)
                obj.xstep(end+1,:)=outputs.xstep;
                obj.ystep(end+1,:)=outputs.ystep;
                obj.zstep(end+1,:)=outputs.zstep;
            end  
            
            obj.xzmp(end+1,:)=outputs.xzmp;
            obj.yzmp(end+1,:)=outputs.yzmp;
            obj.zzmp(end+1,:)=outputs.zzmp;
        end
        
         function obj=add_storage_sensors(obj,outputs) 
            %Stores the properties value of the next iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/add_storage is a function.
            %    obj = add_storage(obj, outputs)
            %
            %Stores the properties value of 'outputs'
            %'outputs' is a classdef_MPC_problem_outputs object
            %
            %all properties, but QP_result_all, are stored as vector
            %QP_result_all is stored as array of cells accessible with {}
            %instead of ()
            
            noise_type = 'rand'; %Uniformly distributed pseudorandom numbers
%             noise_type = 'awgn'; %white Gaussian noise to a signal.
            switch(noise_type)
                case 'awgn'
                    awgn_xc = awgn([obj.xc; outputs.xc],1e-19,'measured');
                    obj.xc(end+1,:)=awgn_xc(end);
                    
%                     obj.xdc(end+1,:)=awgn(outputs.xdc,10,'measured');
%                     obj.xddc(end+1,:)=awgn(outputs.xddc,10,'measured');
%  
%                     obj.yc(end+1,:)=awgn(outputs.yc,10,'measured');
%                     obj.ydc(end+1,:)=awgn(outputs.ydc,10,'measured');
%                     obj.yddc(end+1,:)=awgn(outputs.yddc,10,'measured');
% 
%                     obj.zc(end+1,:)=awgn(outputs.zc,10,'measured');
%                     obj.zdc(end+1,:)=awgn(outputs.zdc,10,'measured');
%                     obj.zddc(end+1,:)=awgn(outputs.zddc,10,'measured');
                    obj.xdc(end+1,:)=outputs.xdc+0.0*(-1 + 2*rand);
                    obj.xddc(end+1,:)=outputs.xddc+0.0*(-1 + 2*rand);
 
                    obj.yc(end+1,:)=outputs.yc+0.00*(-1 + 2*rand);
                    obj.ydc(end+1,:)=outputs.ydc+0.0*(-1 + 2*rand);
                	obj.yddc(end+1,:)=outputs.yddc+0.0*(-1 + 2*rand);
 
                    obj.zc(end+1,:)=outputs.zc+0.0*(-1 + 2*rand);
                    obj.zdc(end+1,:)=outputs.zdc+0.0*(-1 + 2*rand);
                    obj.zddc(end+1,:)=outputs.zddc+0.0*(-1 + 2*rand);
                    
                case 'rand'
                    obj.xc(end+1,:)=outputs.xc+0.001*(-1 + 2*rand);
                    obj.xdc(end+1,:)=outputs.xdc+0.0*(-1 + 2*rand);
                    obj.xddc(end+1,:)=outputs.xddc+0.0*(-1 + 2*rand);
 
                    obj.yc(end+1,:)=outputs.yc+0.001*(-1 + 2*rand);
                    obj.ydc(end+1,:)=outputs.ydc+0.0*(-1 + 2*rand);
                	obj.yddc(end+1,:)=outputs.yddc+0.0*(-1 + 2*rand);
 
                    obj.zc(end+1,:)=outputs.zc+0.001*(-1 + 2*rand);
                    obj.zdc(end+1,:)=outputs.zdc+0.0*(-1 + 2*rand);
                    obj.zddc(end+1,:)=outputs.zddc+0.0*(-1 + 2*rand);
            end
            if ~isempty(outputs.xstep)
                obj.xstep(end+1,:)=outputs.xstep;
                obj.ystep(end+1,:)=outputs.ystep;
                obj.zstep(end+1,:)=outputs.zstep;
            end 
            
            obj.xzmp(end+1,:)=outputs.xzmp;
            obj.yzmp(end+1,:)=outputs.yzmp;
            obj.zzmp(end+1,:)=outputs.zzmp;
        end
        %%  
        
        function obj = physical_model_iteration(obj,experiment,MPC_outputs,physical_model_storage,k)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x_states = zeros(3,1);
            y_states = zeros(3,1);
            z_states = zeros(3,1);
            
            x_states(1)=physical_model_storage.xc(end);
            x_states(2)=physical_model_storage.xdc(end); 
            x_states(3)=physical_model_storage.xddc(end); 
             
            y_states(1)=physical_model_storage.yc(end);
            y_states(2)=physical_model_storage.ydc(end);
            y_states(3)=physical_model_storage.yddc(end); 
             
            z_states(1)=physical_model_storage.zc(end);
            z_states(2)=physical_model_storage.zdc(end);
            z_states(3)=physical_model_storage.zddc(end); 
              
            obj.zzmp = 0;
            obj.xzmp=1*MPC_outputs.xc+0*MPC_outputs.xdc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.xddc;
            obj.yzmp=1*MPC_outputs.yc+0*MPC_outputs.ydc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.yddc;
  
            % ddx= (experiment.g-zddc(k))/zc(k)*(x -zmp)
            % z1 = x
            % z2 = dx =dz1
            % z3 = dz2

            % dz1 = z2
            % dz2 = (experiment.g-zddc(k))/zc(k)*z1 -(experiment.g-zddc(k))/zc(k)*zmp 

            A = [0,1;(experiment.g-physical_model_storage.zddc(end))/physical_model_storage.zc(end),0]; 
            B = [0;-(experiment.g-physical_model_storage.zddc(end))/physical_model_storage.zc(end)]; 
            C = [(experiment.g-physical_model_storage.zddc(end))/physical_model_storage.zc(end),0];
            D = [-(experiment.g-physical_model_storage.zddc(end))/physical_model_storage.zc(end)];

            % Create state space model
            d_sys = ss(A,B,C,D);

            Ts = experiment.phase_duration_sampling(k);
            d_sys = c2d(d_sys,Ts);
            % DelayT(1) = struct('delay',1,'a',zeros(size(A)),'b',zeros(size(B)),'c',C,'d',D);
            % % DelayT(2) = struct('delay',1.2,'a',-1,'b',0,'c',0,'d',0);
            % sys2 = delayss(A,B,zeros(size(C)),zeros(size(D)),DelayT); 
            % % Convert to discrete, where dt is your discrete time-step (in seconds)
            % % d_sys = c2d(sys,Ts);
%             sys_Delay = ss(A,B,C,D,'OutputDelay',2.5);
%             sys_Delay = ss(A,B,C,D,'OutputDelay',0);
%             d_sys = sys_Delay;

            dA = d_sys.A; 
            dB = d_sys.B; 
            dC = d_sys.C;
            dD = d_sys.D;

            ux = obj.xzmp;
            uy = obj.yzmp;

            % Repeat application of Euler method sampled at Ts/M.
            M = 10;
            delta = Ts/M; 
%             for dt=1:M
%                 x_states(1:2) = x_states(1:2) + delta*(dA*x_states(1:2) + dB*ux);
%                 y_states(1:2) = y_states(1:2) + delta*(dA*y_states(1:2) + dB*uy);
%                 x_states(3) = x_states(3) + delta*( dC*x_states(1:2)+ dD*ux);
%                 y_states(3) = y_states(3) + delta*(dC*y_states(1:2)+ dD*uy);
%             end 
            for dt=1:M
                x_states(1:2) = x_states(1:2) + delta*(A*x_states(1:2) + B*ux);
                y_states(1:2) = y_states(1:2) + delta*(A*y_states(1:2) + B*uy);
                x_states(3) = x_states(3) + delta*(C*x_states(1:2)+ D*ux);
                y_states(3) = y_states(3) + delta*(C*y_states(1:2)+ D*uy);
            end  
            
%             x_states(3) = dC*x_states(1:2)+ dD*ux;
%             y_states(3) = dC*y_states(1:2)+ dD*uy;
 
            obj.xc=x_states(1);
            obj.xdc=x_states(2); 
            obj.xddc=x_states(3);
             
            obj.yc=y_states(1);
            obj.ydc=y_states(2);
            obj.yddc=y_states(3);
             
            obj.zc=z_states(1);
            obj.zdc=z_states(2);
            obj.zddc=z_states(3);  
             
%             if ~isempty(MPC_outputs.xstep)
%                 obj.xstep(end+1,:)=MPC_outputs.xstep;
%                 obj.ystep(end+1,:)=MPC_outputs.ystep;
%                 obj.zstep(end+1,:)=MPC_outputs.zstep;
%             end 
            
            



        end%%  
        
        function obj = physical_model_iteration2(obj,experiment,MPC_outputs,physical_model_storage,k)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
             
            obj.zzmp= 0;
            obj.xzmp=MPC_outputs.xc+0*MPC_outputs.xdc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.xddc;
            obj.yzmp=MPC_outputs.yc+0*MPC_outputs.ydc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.yddc;
   
            x_states = function_lip(physical_model_storage.xc(end),...
                                       physical_model_storage.xdc(end),...
                                       physical_model_storage.zc(end),...
                                       obj.xzmp,...
                                       experiment.phase_duration_sampling_cumul(k),...
                                       experiment.g,experiment.phase_duration_sampling(k));
              
            y_states = function_lip(physical_model_storage.yc(end),...
                                       physical_model_storage.ydc(end),...
                                       physical_model_storage.zc(end),...
                                       obj.yzmp,...
                                       experiment.phase_duration_sampling_cumul(k),...
                                       experiment.g,...
                                       experiment.phase_duration_sampling(k));
                             
            obj.xc=x_states(1);
            obj.xdc=x_states(2); 
            obj.xddc=x_states(3);
             
            obj.yc=y_states(1);
            obj.ydc=y_states(2);
            obj.yddc=y_states(3);
             
            obj.zc=physical_model_storage.zc(end);
            obj.zdc=physical_model_storage.zdc(end);
            obj.zddc=physical_model_storage.zddc(end);  
              
        end
        function obj = physical_model_iteration3(obj,experiment,MPC_outputs,physical_model_storage,k)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
             
            obj.zzmp= 0;
            obj.xzmp=MPC_outputs.xc+0*MPC_outputs.xdc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.xddc;
            obj.yzmp=MPC_outputs.yc+0*MPC_outputs.ydc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.yddc;
   
            x_states = function_3dlip(physical_model_storage.xc(end),...
                                       physical_model_storage.xdc(end),...
                                       physical_model_storage.zc(end),...
                                       obj.xzmp,...
                                       experiment.phase_duration_sampling_cumul(k),...
                                       experiment.g,experiment.phase_duration_sampling(k));
              
            y_states = function_3dlip(physical_model_storage.yc(end),...
                                       physical_model_storage.ydc(end),...
                                       physical_model_storage.zc(end),...
                                       obj.yzmp,...
                                       experiment.phase_duration_sampling_cumul(k),...
                                       experiment.g,...
                                       experiment.phase_duration_sampling(k));
                             
            obj.xc=x_states(1);
            obj.xdc=x_states(2); 
            obj.xddc=x_states(3);
             
            obj.yc=y_states(1);
            obj.ydc=y_states(2);
            obj.yddc=y_states(3);
             
            obj.zc=physical_model_storage.zc(end);
            obj.zdc=physical_model_storage.zdc(end);
            obj.zddc=physical_model_storage.zddc(end);  
              
        end
        function obj = physical_model_iteration_F_lin(obj,experiment,MPC_outputs,physical_model_storage,k)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
             
            obj.zzmp= 0;
            obj.xzmp=MPC_outputs.xc+0*MPC_outputs.xdc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.xddc;
            obj.yzmp=MPC_outputs.yc+0*MPC_outputs.ydc-(MPC_outputs.zc-obj.zzmp)/(MPC_outputs.zddc+experiment.g).*MPC_outputs.yddc;
   
            Ts=experiment.phase_duration_sampling(k); 
            M=50;
            delta=Ts/M; 
            tspan=0:delta:Ts;
            xzmp_xc=obj.xzmp-physical_model_storage.xc(end);
            L = 0.780678;
            x0=[atan(xzmp_xc/L);physical_model_storage.xdc(end)] ;
            %% SIMULATION:
            % simulate dynamics of pendulum free oscilation (nonlinear)
%             [T, S] = ode45(@(t,y) F_nonlin(t,y,L),tspan,x0);
            % simulate dynamics of pendulum free oscilation (nonlinear) 
            [T_L, S_L] = ode45(@(t,y) F_lin(t,y,L),tspan,x0); % could use lsim instead
            x_states = zeros(3,1);
            x_states(1) = L*sin(S_L(end,1));
            x_states(2) = L*S_L(end,2);
            omega = sqrt(physical_model_storage.zc(end)/experiment.g);
            x_states(3) = omega^2*(xzmp_xc);  
            
            
            
            
            
            yzmp_yc=obj.yzmp-physical_model_storage.yc(end);
            L = 0.780678;
            x0=[atan(yzmp_yc/L);physical_model_storage.ydc(end)] ;
            %% SIMULATION:
            % simulate dynamics of pendulum free oscilation (nonlinear)
%             [T, S] = ode45(@(t,y) F_nonlin(t,y,L),tspan,x0);
            % simulate dynamics of pendulum free oscilation (nonlinear) 
            [T_L, S_L] = ode45(@(t,y) F_lin(t,y,L),tspan,x0); % could use lsim instead
            y_states = zeros(3,1);
            y_states(1) = L*sin(S_L(end,1));
            y_states(2) = L*S_L(end,2);
            omega = sqrt(physical_model_storage.zc(end)/experiment.g);
            y_states(3) = omega^2*(yzmp_yc);  
            
            
            
            
%             
%             x_states = function_3dlip(physical_model_storage.xc(end),...
%                                        physical_model_storage.xdc(end),...
%                                        physical_model_storage.zc(end),...
%                                        obj.xzmp,...
%                                        experiment.phase_duration_sampling_cumul(k),...
%                                        experiment.g,experiment.phase_duration_sampling(k));
%               
%             y_states = function_3dlip(physical_model_storage.yc(end),...
%                                        physical_model_storage.ydc(end),...
%                                        physical_model_storage.zc(end),...
%                                        obj.yzmp,...
%                                        experiment.phase_duration_sampling_cumul(k),...
%                                        experiment.g,...
%                                        experiment.phase_duration_sampling(k));
%                              
            obj.xc=x_states(1);
            obj.xdc=x_states(2); 
            obj.xddc=x_states(3);
             
            obj.yc=y_states(1);
            obj.ydc=y_states(2);
            obj.yddc=y_states(3);
             
            obj.zc=physical_model_storage.zc(end);
            obj.zdc=physical_model_storage.zdc(end);
            obj.zddc=physical_model_storage.zddc(end);  
              
        end
    end
end

