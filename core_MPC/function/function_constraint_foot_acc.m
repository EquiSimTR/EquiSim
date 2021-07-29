function [A,b]=function_constraint_foot_acc(MPC_inputs,Step_state_preview,xstep,ystep)
%% Constraint foot acceleration
    A_foot=[];
%     b_foot_ddx=[];
    b_foot=[];
%     yb_step_stretch=[];
    A_foot_ddx=[];
    b_foot_ddx=[];
%     t_i =0.0:0.001:MPC_inputs.phase_duration_r;
    t_i =0.0:0.001:.33;
%     t_i =0.01:0.01:0.320;
    t_f =t_i(end); 
     xddankmax = 239;
%      xddankmax = 1; 
    xddankmin =-xddankmax;
    
    for j=1:length(t_i)  
        
        b_acc =(120*t_i(j)^3/t_f^5-180*t_i(j)^2/t_f^4+60*t_i(j)/t_f^3); 
        
        if size(Step_state_preview.Pu_step,2)>=1
            A_foot=eye(size(Step_state_preview.Pu_step,2));
            A_foot(2:end,1:end-1)=A_foot(2:end,1:end-1)-eye(size(Step_state_preview.Pu_step,2)-1);
            A_foot=[zeros(size(Step_state_preview.Pu_step,2),MPC_inputs.N) A_foot];
            A_foot=b_acc*[A_foot;-A_foot];
             
    %         A_foot=blkdiag(A_foot,A_foot);
            A_foot=blkdiag(A_foot);
            A_foot_ddx=[A_foot_ddx;A_foot];
            
            
            %%%
            phase_type_nodouble_reduce=MPC_inputs.phase_type_reduce(any(MPC_inputs.phase_type_reduce~='b'&MPC_inputs.phase_type_reduce~="start"&MPC_inputs.phase_type_reduce~="stop",2));

            if MPC_inputs.phase_type_reduce(end)=="stop" && Step_state_preview.Pu_step(end,end)==0.5
                if phase_type_nodouble_reduce(end)=='r'
                    phase_type_nodouble_reduce(end+1)='l';
                else
                    phase_type_nodouble_reduce(end+1)='r';
                end
            end

            phase_type_nodouble_reduce(1)=[];
            %%%
            b_foot=[ones(size(Step_state_preview.Pu_step,2),1)*xddankmax;...
                ones(size(Step_state_preview.Pu_step,2),1)*(-xddankmin)];
            b_foot([1 end/2+1])=b_foot([1 end/2+1])+b_acc*[xstep(end);-xstep(end)];

    %         yb_step_stretch=[any(phase_type_nodouble_reduce=='l',2)*MPC_inputs.yankmax-any(phase_type_nodouble_reduce=='r',2)*MPC_inputs.yankmin;...
    %             any(phase_type_nodouble_reduce=='l',2)*(-MPC_inputs.yankmin)-any(phase_type_nodouble_reduce=='r',2)*(-MPC_inputs.yankmax)];
    %         yb_step_stretch([1 end/2+1])=yb_step_stretch([1 end/2+1])+[ystep(end);-ystep(end)];
        end

    %     b_foot_ddx=[b_foot;yb_step_stretch];
        b_foot_ddx=[b_foot_ddx; b_foot];
        
    end

        A=A_foot_ddx;
        b=b_foot_ddx;