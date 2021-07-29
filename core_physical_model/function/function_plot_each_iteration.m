% function function_plot_each_iteration(MPC_outputs_storage,physical_model_storage, sensor_dynamics_storage,... 
%                        ~,varargin{4},~,varargin{6},~,senmdl_com_on_off,~,cop_on_off,...
%                        ~,x_on_off,~,y_on_off,~,z_on_off,~,time_on_off, ...
%                        ~,drawing_feet_steps_on_off,experiment,k)
%     
function function_plot_each_iteration(MPC_outputs_storage,physical_model_storage,sensor_dynamics_storage,experiment,varargin)                       
k=varargin{end};
%checks the input  
contains_on = ismember('on',varargin(1:8));
%checks the input
switch contains_on
    case 1 
        switch varargin{2}
            case 'on'
                if varargin{12}
                    figure(1)
                    clf 
                    % title('trajectories 3D')
                    xlabel('time [s]') % x-axis label
                    ylabel('x [m]') % y-axis label
                    axis auto
                    % axis image
                    hold on      
                    d=plot(experiment.phase_duration_sampling_cumul(1:k+1),physical_model_storage.xzmp,'-r');
                    set(d,'Visible',varargin{10}); %'off' or 'on' 
                    b=plot(experiment.phase_duration_sampling_cumul(1:k+1),physical_model_storage.xc,'-b');
                    set(b,'Visible',varargin{6}); %'off' or 'on' 
                    c=plot(experiment.phase_duration_sampling_cumul(1:k+1),sensor_dynamics_storage.xc,'-g');
                    set(c,'Visible',varargin{8}); %'off' or 'on'
                    a=plot(experiment.phase_duration_sampling_cumul(1:k+1),MPC_outputs_storage.xc,'-k');
                    set(a,'Visible',varargin{4}); %'off' or 'on'
                    hold off 
                    legend('COP from MPC output','COM from physical model','COM from sensors','COM from MPC output','Location','southeast')
                end
                if varargin{14}
                    figure(2)
                    clf 
                    % title('trajectories 3D')
                    xlabel('time [s]') % x-axis label
                    ylabel('y [m]') % y-axis label
                    axis auto
                    % axis image
                    hold on      
                    d=plot(experiment.phase_duration_sampling_cumul(1:k+1),physical_model_storage.yzmp,'-r');
                    set(d,'Visible',varargin{10}); %'off' or 'on' 
                    b=plot(experiment.phase_duration_sampling_cumul(1:k+1),physical_model_storage.yc,'-b');
                    set(b,'Visible',varargin{6}); %'off' or 'on' 
                    c=plot(experiment.phase_duration_sampling_cumul(1:k+1),sensor_dynamics_storage.yc,'-g');
                    set(c,'Visible',varargin{8}); %'off' or 'on'
                    a=plot(experiment.phase_duration_sampling_cumul(1:k+1),MPC_outputs_storage.yc,'-k');
                    set(a,'Visible',varargin{4}); %'off' or 'on'
                    hold off 
                    legend('COP from MPC output','COM from physical model','COM from sensors','COM from MPC output','Location','southeast')
                end 
                if varargin{15}
                    figure(3)
                    clf  
                    % title('trajectories 3D')
                    xlabel('time [s]') % x-axis label
                    ylabel('z [m]') % y-axis label
                    axis auto
                    % axis image
                    hold on       
                    b=plot(experiment.phase_duration_sampling_cumul(1:k+1),physical_model_storage.zc,'-b');
                    set(b,'Visible',varargin{6}); %'off' or 'on' 
                    c=plot(experiment.phase_duration_sampling_cumul(1:k+1),sensor_dynamics_storage.zc,'-g');
                    set(c,'Visible',varargin{8}); %'off' or 'on'
                    a=plot(experiment.phase_duration_sampling_cumul(1:k+1),MPC_outputs_storage.zc,'-k');
                    set(a,'Visible',varargin{4}); %'off' or 'on'
                    hold off 
                    legend('COM from physical model','COM from sensors','COM from MPC output','Location','southeast')
                end
            case 'off' 
                figure(1)
                clf  
                hold on
                d_=plot(physical_model_storage.xzmp,physical_model_storage.yzmp,'-r');
                set(d_,'Visible',varargin{10}); %'off' or 'on'
                b=plot(physical_model_storage.xc,physical_model_storage.yc,'-b');
                set(b,'Visible',varargin{6}); %'off' or 'on' 
                c=plot(sensor_dynamics_storage.xc,sensor_dynamics_storage.yc,'-g');
                set(c,'Visible',varargin{8}); %'off' or 'on' 
                a(1)=plot(MPC_outputs_storage.xc,MPC_outputs_storage.yc,'-k');
                set(a,'Visible',varargin{4}); %'off' or 'on'  
                hold off 
                % title('trajectories 3D')
                xlabel('x [m]') % x-axis label
                ylabel('y [m]') % y-axis label
                axis auto % axis image
                legend('COP from MPC output','COM from physical model','COM from sensors','COM from MPC output','Location','southeast')
        end
    otherwise
        % errors
        %error('what are you going to plot?.')

end
                         
                             
                             
                             
% 	if onoff(mpc_on_off)
%         figure(1);
%         clc
%         clf  
%         axis([0,size(b,1),-1,12])
%         hold on
%         plot(A*obj.QP_result_all+b,'k-','LineWidth',2)  
%     end
    
    
    
    
    
    
end

