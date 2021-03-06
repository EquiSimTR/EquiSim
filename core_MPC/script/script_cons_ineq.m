%% Constraints inequalities
    %TODO orientation

%% Constraint ZMP in convex hull
    [A_zmp,b_zmp]=function_constraint_convexhull(...
    CoP_state_preview.Pu_z_up,CoP_state_preview.Pu_z_down,Step_state_preview.Pu_step,...
    CoP_state_preview.f_z_up(:,1),CoP_state_preview.f_z_down(:,1),Step_state_preview.f_step(:,1),...
    CoP_state_preview.f_z_up(:,2),CoP_state_preview.f_z_down(:,2),Step_state_preview.f_step(:,2),...
    MPC_inputs.no_double_support,MPC_inputs.double_support,MPC_inputs.right_support,MPC_inputs.left_support,...
    MPC_inputs.fronttoankle,MPC_inputs.backtoankle,MPC_inputs.inttoankle,MPC_inputs.exttoankle,MPC_inputs.sole_margin,...
    MPC_inputs.yaw);
        
    %% Constraint foot step stretching
    [A_step_stretch,b_step_stretch]=function_constraint_foot_stretching(MPC_inputs,Step_state_preview,MPC_inputs.xstep,MPC_inputs.ystep);
    
    % Constraint concatenation
    A=[A_zmp;A_step_stretch*0];
    b=[b_zmp;b_step_stretch*0];        
    
    A=[A zeros(size(A,1),size(COM_state_preview.Pu_c,2))];    
    %% Constraint vertical com motion
    A_verti_motion_up=COM_state_preview.Pu_c-diag(MPC_inputs.zeta_up)*COM_state_preview.Pu_ddc;
    b_verti_motion_up=MPC_inputs.zeta_up.*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce-(COM_state_preview.f_c(:,3)-MPC_inputs.zeta_up.*COM_state_preview.f_ddc(:,3));

    A_verti_motion_down=COM_state_preview.Pu_c-diag(MPC_inputs.zeta_down)*COM_state_preview.Pu_ddc;
    b_verti_motion_down=MPC_inputs.zeta_down.*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce-(COM_state_preview.f_c(:,3)-MPC_inputs.zeta_down.*COM_state_preview.f_ddc(:,3));

    
    
    if 0
        A_verti_motion=[A_verti_motion_up(2:end,:)*0;-A_verti_motion_down(2:end,:)*0];
        b_verti_motion=[b_verti_motion_up(2:end,:)*0;-b_verti_motion_down(2:end,:)*0];
    else
        A_verti_motion=[A_verti_motion_up;-A_verti_motion_down];
        b_verti_motion=[b_verti_motion_up;-b_verti_motion_down];
    end

    %% COM accel z > -g
    A_verti_acc=-COM_state_preview.Pu_ddc;
    b_verti_acc=MPC_inputs.g+COM_state_preview.f_ddc(:,3);    
    
    %%
    A_verti=[A_verti_motion;A_verti_acc];
    b_verti=[b_verti_motion;b_verti_acc];
    
    A=[A;zeros(size(A_verti,1),size(A_zmp,2)) A_verti];
    b=[b;b_verti];
    
    
    %% constraint kinematics com height (polyhedron)
    switch MPC_inputs.kinematic_limit
        case ''
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron(...
            COM_state_preview.Pu_c,Step_state_preview.Pu_step,...
            COM_state_preview.f_c(:,1),Step_state_preview.f_step(:,1),...
            COM_state_preview.f_c(:,2),Step_state_preview.f_step(:,2),...
            COM_state_preview.f_c(:,3),MPC_inputs.zfloor_ref_reduce,...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron_hexagon(...
            COM_state_preview.Pu_c,Step_state_preview.Pu_step,...
            COM_state_preview.f_c(:,1),Step_state_preview.f_step(:,1),...
            COM_state_preview.f_c(:,2),Step_state_preview.f_step(:,2),...
            COM_state_preview.f_c(:,3),MPC_inputs.zfloor_ref_reduce,...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,h_com+h_com_min);
        case 'hexagonTranslation'
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron_hexagon_translation(...
            COM_state_preview.Pu_c,Step_state_preview.Pu_step,...
            COM_state_preview.f_c(:,1),Step_state_preview.f_step(:,1),...
            COM_state_preview.f_c(:,2),Step_state_preview.f_step(:,2),...
            COM_state_preview.f_c(:,3),MPC_inputs.zfloor_ref_reduce,...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,MPC_inputs.z_leg_min+MPC_inputs.z_decalage_tot,MPC_inputs.translate_step_polyhedron_type(1,1),MPC_inputs.translate_step_polyhedron_type(1,2),...
            MPC_inputs.left_support,MPC_inputs.right_support);
        otherwise
%             MPC_inputs.left_support
%             MPC_inputs.right_support
            error('choose a type of kinematic_limit')
    end

    A=[A;A_diff_c_p];
    b=[b;b_diff_c_p];

%% Constraint Last Capture point in convex hull
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='r'
        right_support_capture=any(sum(MPC_inputs.Px_step(:,1:2:end)==1,2),2);
        left_support_capture=any(sum(MPC_inputs.Px_step(:,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='l'
        right_support_capture=any(sum(MPC_inputs.Px_step(:,2:2:end)==1,2),2);
        left_support_capture=any(sum(MPC_inputs.Px_step(:,1:2:end)==1,2),2);
    else
        right_support_capture=false(size(MPC_inputs.no_double_support_capture));
        left_support_capture=false(size(MPC_inputs.no_double_support_capture));
    end
        
    switch(MPC_inputs.COM_form)
        case {'comPolynomial','comExponential'}
                [A_Capture,b_Capture]=function_constraint_convexhull(...
                    COM_state_preview.Pu_c+COM_state_preview.Pu_dc/MPC_inputs.omega_temp,COM_state_preview.Pu_c+COM_state_preview.Pu_dc/MPC_inputs.omega_temp,Step_state_preview.Pu_step,...
                    COM_state_preview.f_c(:,1)+COM_state_preview.f_dc(:,1)/MPC_inputs.omega_temp,COM_state_preview.f_c(:,1)+COM_state_preview.f_dc(:,1)/MPC_inputs.omega_temp,Step_state_preview.f_step(:,1),...
                    COM_state_preview.f_c(:,2)+COM_state_preview.f_dc(:,2)/MPC_inputs.omega_temp,COM_state_preview.f_c(:,2)+COM_state_preview.f_dc(:,2)/MPC_inputs.omega_temp,Step_state_preview.f_step(:,2),...
                    MPC_inputs.no_double_support_capture,[],MPC_inputs.right_support,MPC_inputs.left_support,...
                    MPC_inputs.fronttoankle,MPC_inputs.backtoankle,MPC_inputs.inttoankle,MPC_inputs.exttoankle,MPC_inputs.sole_margin,...
                    MPC_inputs.yaw);
        case 'comPolyExpo'
                [A_Capture,b_Capture]=function_constraint_convexhull(...
                    COM_state_preview.Pu_c+3/2*COM_state_preview.Pu_dc/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc/MPC_inputs.omega_temp^2,COM_state_preview.Pu_c+3/2*COM_state_preview.Pu_dc/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc/MPC_inputs.omega_temp^2,Step_state_preview.Pu_step,...
                    COM_state_preview.f_c(:,1)+3/2*COM_state_preview.f_dc(:,1)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(:,1)/MPC_inputs.omega_temp^2,COM_state_preview.f_c(:,1)+3/2*COM_state_preview.f_dc(:,1)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(:,1)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(:,1),...
                    COM_state_preview.f_c(:,2)+3/2*COM_state_preview.f_dc(:,2)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(:,2)/MPC_inputs.omega_temp^2,COM_state_preview.f_c(:,2)+3/2*COM_state_preview.f_dc(:,2)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(:,2)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(:,2),...
                    MPC_inputs.no_double_support_capture,[],MPC_inputs.right_support,MPC_inputs.left_support,...
                    MPC_inputs.fronttoankle,MPC_inputs.backtoankle,MPC_inputs.inttoankle,MPC_inputs.exttoankle,MPC_inputs.sole_margin,...
                    MPC_inputs.yaw);
        otherwise
            error('Bad COM_form')
    end

    
    A_Capture=[A_Capture zeros(size(A_Capture,1),size(COM_state_preview.Pu_c,2))];  
    
    if size(A_Capture,1)>0
        A_Capture([3 4 7 8],:)=[];
        b_Capture([3 4 7 8],:)=[];
    end
    
    % Constraint concatenation
    if i>=MPC_inputs.no_end_constraint %|| i==40 %|| (phase_type_sampling_reduce(end-3)~=phase_type_sampling_reduce(end)&&phase_type_sampling_reduce(end-3)~="b"&&phase_type_sampling_reduce(end)~="b")
        A=[A;A_Capture*0];
        b=[b;b_Capture*0]; 
    else
        A=[A;A_Capture*0];
        b=[b;b_Capture*0]; 
    end
       
%% constraint kinematics Last capture point height (polyhedron)
    if isempty(Step_state_preview.Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(1,0);
    else
        Pu_step_temp=Step_state_preview.Pu_step(end,:);
    end
    
    
    switch(MPC_inputs.COM_form)
        case {'comPolynomial','comExponential'}
            switch MPC_inputs.kinematic_limit
                case ''
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron(...
                    COM_state_preview.Pu_c(end,:)+COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp,Pu_step_temp,...
                    COM_state_preview.f_c(end,1)+COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,1),...
                    COM_state_preview.f_c(end,2)+COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,2),...
                    COM_state_preview.f_c(end,3)+COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp,MPC_inputs.zfloor_ref_reduce(end,:),...
                    rot_successive,polyhedron_lim);
                case 'hexagon'
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon(...
                    COM_state_preview.Pu_c(end,:)+COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp,Pu_step_temp,...
                    COM_state_preview.f_c(end,1)+COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,1),...
                    COM_state_preview.f_c(end,2)+COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,2),...
                    COM_state_preview.f_c(end,3)+COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp,MPC_inputs.zfloor_ref_reduce(end,:),...%MPC_inputs.zfloor_ref_reduce
                    MPC_inputs.plan_hexagon,h_com+h_com_min);
                case 'hexagonTranslation'
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon_translation(...
                    COM_state_preview.Pu_c(end,:)+COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp,Pu_step_temp,...
                    COM_state_preview.f_c(end,1)+COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,1),...
                    COM_state_preview.f_c(end,2)+COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp,Step_state_preview.f_step(end,2),...
                    COM_state_preview.f_c(end,3)+COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp,MPC_inputs.zfloor_ref_reduce(end,:),...%MPC_inputs.zfloor_ref_reduce
                    MPC_inputs.plan_hexagon,MPC_inputs.z_leg_min+MPC_inputs.z_decalage_tot,MPC_inputs.translate_step_polyhedron_type(1,1),MPC_inputs.translate_step_polyhedron_type(1,2),...
                    MPC_inputs.left_support(end,:),MPC_inputs.right_support(end,:));
                otherwise
                    error('choose a type of kinematic_limit')
            end
        case 'comPolyExpo'
                switch MPC_inputs.kinematic_limit
                    case ''
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron(...
                        COM_state_preview.Pu_c(end,:)+3/2*COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc(end,:)/MPC_inputs.omega_temp^2,Pu_step_temp,...
                        COM_state_preview.f_c(end,1)+3/2*COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,1)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,1),...
                        COM_state_preview.f_c(end,2)+3/2*COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,2)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,2),...
                        COM_state_preview.f_c(end,3)+3/2*COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,3)/MPC_inputs.omega_temp^2,MPC_inputs.zfloor_ref_reduce(end,:),...%MPC_inputs.zfloor_ref_reduce
                        rot_successive,polyhedron_lim);
                    case 'hexagon'
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon(...
                        COM_state_preview.Pu_c(end,:)+3/2*COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc(end,:)/MPC_inputs.omega_temp^2,Pu_step_temp,...
                        COM_state_preview.f_c(end,1)+3/2*COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,1)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,1),...
                        COM_state_preview.f_c(end,2)+3/2*COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,2)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,2),...
                        COM_state_preview.f_c(end,3)+3/2*COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,3)/MPC_inputs.omega_temp^2,MPC_inputs.zfloor_ref_reduce(end,:),...%MPC_inputs.zfloor_ref_reduce
                        MPC_inputs.plan_hexagon,h_com+h_com_min);
                    case 'hexagonTranslation'
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon_translation(...
                        COM_state_preview.Pu_c(end,:)+3/2*COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc(end,:)/MPC_inputs.omega_temp^2,Pu_step_temp,...
                        COM_state_preview.f_c(end,1)+3/2*COM_state_preview.f_dc(end,1)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,1)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,1),...
                        COM_state_preview.f_c(end,2)+3/2*COM_state_preview.f_dc(end,2)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,2)/MPC_inputs.omega_temp^2,Step_state_preview.f_step(end,2),...
                        COM_state_preview.f_c(end,3)+3/2*COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,3)/MPC_inputs.omega_temp^2,MPC_inputs.zfloor_ref_reduce(end,:),...%MPC_inputs.zfloor_ref_reduce
                        MPC_inputs.plan_hexagon,MPC_inputs.z_leg_min+MPC_inputs.z_decalage_tot,MPC_inputs.translate_step_polyhedron_type(1,1),MPC_inputs.translate_step_polyhedron_type(1,2),...
                        MPC_inputs.left_support(end,:),MPC_inputs.right_support(end,:));
                    otherwise
                        error('choose a type of kinematic_limit')
                end
        otherwise
            error('Bad COM_form')
    end

    
    if i>=MPC_inputs.no_end_constraint %|| i==40
        A=[A;A_diff_capture_p*0];
        b=[b;b_diff_capture_p*0];
    else
        A=[A;A_diff_capture_p*0];
        b=[b;b_diff_capture_p*0];
    end
    
%% Constraint vertical com motion on horizon
    switch(MPC_inputs.COM_form)
        case {'comPolynomial','comExponential'}
            A_verti_motion_up_horizon=COM_state_preview.Pu_c(end,:)+COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp;
            b_verti_motion_up_horizon=MPC_inputs.zeta_up(end,:).*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce(end,:)-(COM_state_preview.f_c(end,3)+COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp);

            A_verti_motion_down_horizon=COM_state_preview.Pu_c(end,:)+COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp;
            b_verti_motion_down_horizon=MPC_inputs.zeta_down(end,:).*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce(end,:)-(COM_state_preview.f_c(end,3)+COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp);

            A_verti_motion_horizon=[A_verti_motion_up_horizon;-A_verti_motion_down_horizon];
            b_verti_motion_horizon=[b_verti_motion_up_horizon;-b_verti_motion_down_horizon];        
        case 'comPolyExpo'
            A_verti_motion_up_horizon=COM_state_preview.Pu_c(end,:)+3/2*COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc(end,:)/MPC_inputs.omega_temp^2;
            b_verti_motion_up_horizon=MPC_inputs.zeta_up(end,:).*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce(end,:)-(COM_state_preview.f_c(end,3)+3/2*COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,3)/MPC_inputs.omega_temp^2);

            A_verti_motion_down_horizon=COM_state_preview.Pu_c(end,:)+3/2*COM_state_preview.Pu_dc(end,:)/MPC_inputs.omega_temp+1/2*COM_state_preview.Pu_ddc(end,:)/MPC_inputs.omega_temp^2;
            b_verti_motion_down_horizon=MPC_inputs.zeta_down(end,:).*MPC_inputs.g+MPC_inputs.zfloor_ref_reduce(end,:)-(COM_state_preview.f_c(end,3)+3/2*COM_state_preview.f_dc(end,3)/MPC_inputs.omega_temp+1/2*COM_state_preview.f_ddc(end,3)/MPC_inputs.omega_temp^2);

            A_verti_motion_horizon=[A_verti_motion_up_horizon;-A_verti_motion_down_horizon];
            b_verti_motion_horizon=[b_verti_motion_up_horizon;-b_verti_motion_down_horizon];
        otherwise
            error('Bad COM_form')
    end

    
    
    
    A=[A;zeros(size(A_verti_motion_horizon,1),size(A_zmp,2)) A_verti_motion_horizon];
    b=[b;b_verti_motion_horizon];
    
    
%% constraint kinematics COM height (polyhedron) beforte and after DSP
    double_support_before=[any([MPC_inputs.phase_type_sampling(1:end-1)=='b']+[MPC_inputs.phase_type_sampling(2:end)~='b']==2,2);false];
    double_support_after=[false;any([MPC_inputs.phase_type_sampling(1:end-1)=='b']+[MPC_inputs.phase_type_sampling(2:end)~='b']==2,2)];
    
    if isempty(Step_state_preview.Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(sum(double_support_after),0);
    else
        Pu_step_temp=Step_state_preview.Pu_step(double_support_after,:);
    end
    
    switch MPC_inputs.kinematic_limit
        case ''
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron(...
            COM_state_preview.Pu_c(double_support_before,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_before,1),Step_state_preview.f_step(double_support_after,1),...
            COM_state_preview.f_c(double_support_before,2),Step_state_preview.f_step(double_support_after,2),...
            COM_state_preview.f_c(double_support_before,3),MPC_inputs.zfloor_ref_reduce(double_support_after,:),...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron_hexagon(...
            COM_state_preview.Pu_c(double_support_before,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_before,1),Step_state_preview.f_step(double_support_after,1),...
            COM_state_preview.f_c(double_support_before,2),Step_state_preview.f_step(double_support_after,2),...
            COM_state_preview.f_c(double_support_before,3),MPC_inputs.zfloor_ref_reduce(double_support_after,:),...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,h_com+h_com_min);
        case 'hexagonTranslation'
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron_hexagon_translation(...
            COM_state_preview.Pu_c(double_support_before,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_before,1),Step_state_preview.f_step(double_support_after,1),...
            COM_state_preview.f_c(double_support_before,2),Step_state_preview.f_step(double_support_after,2),...
            COM_state_preview.f_c(double_support_before,3),MPC_inputs.zfloor_ref_reduce(double_support_after,:),...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,MPC_inputs.z_leg_min+MPC_inputs.z_decalage_tot,MPC_inputs.translate_step_polyhedron_type(1,1),MPC_inputs.translate_step_polyhedron_type(1,2),...
            MPC_inputs.left_support(double_support_after,:),MPC_inputs.right_support(double_support_after,:));
        otherwise
            error('choose a type of kinematic_limit')
    end
    

    A=[A;A_diff_c_p_DSP_before*0];
    b=[b;b_diff_c_p_DSP_before*0];
    
    
    double_support_before=[any([MPC_inputs.phase_type_sampling(1:end-1)=='b']+[MPC_inputs.phase_type_sampling(2:end)~='b']==2,2);false];
    double_support_after=[false;any([MPC_inputs.phase_type_sampling(1:end-1)=='b']+[MPC_inputs.phase_type_sampling(2:end)~='b']==2,2)];
    
    if isempty(Step_state_preview.Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(sum(double_support_before),0);
    else
        Pu_step_temp=Step_state_preview.Pu_step(double_support_before,:);
    end
    
    switch MPC_inputs.kinematic_limit
        case ''
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron(...
            COM_state_preview.Pu_c(double_support_after,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_after,1),Step_state_preview.f_step(double_support_before,1),...
            COM_state_preview.f_c(double_support_after,2),Step_state_preview.f_step(double_support_before,2),...
            COM_state_preview.f_c(double_support_after,3),MPC_inputs.zfloor_ref_reduce(double_support_before,:),...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron_hexagon(...
            COM_state_preview.Pu_c(double_support_after,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_after,1),Step_state_preview.f_step(double_support_before,1),...
            COM_state_preview.f_c(double_support_after,2),Step_state_preview.f_step(double_support_before,2),...
            COM_state_preview.f_c(double_support_after,3),MPC_inputs.zfloor_ref_reduce(double_support_before,:),...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,h_com+h_com_min);
         case 'hexagonTranslation'
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron_hexagon_translation(...
            COM_state_preview.Pu_c(double_support_after,:),Pu_step_temp,...
            COM_state_preview.f_c(double_support_after,1),Step_state_preview.f_step(double_support_before,1),...
            COM_state_preview.f_c(double_support_after,2),Step_state_preview.f_step(double_support_before,2),...
            COM_state_preview.f_c(double_support_after,3),MPC_inputs.zfloor_ref_reduce(double_support_before,:),...%MPC_inputs.zfloor_ref_reduce
            MPC_inputs.plan_hexagon,MPC_inputs.z_leg_min+MPC_inputs.z_decalage_tot,MPC_inputs.translate_step_polyhedron_type(1,1),MPC_inputs.translate_step_polyhedron_type(1,2),...
            MPC_inputs.left_support(double_support_before,:),MPC_inputs.right_support(double_support_before,:));
        otherwise
            error('choose a type of kinematic_limit')
    end
    
    A=[A;A_diff_c_p_DSP_after*0];
    b=[b;b_diff_c_p_DSP_after*0];

[A_xdd_step,b_xdd_step]=function_constraint_foot_acc(MPC_inputs,Step_state_preview,MPC_inputs.xstep,MPC_inputs.ystep);
        
A_xdd_step=[A_xdd_step zeros(size(A_xdd_step,1),size(A,2)-size(A_xdd_step,2))];
 
     
A=[A;A_xdd_step];
b=[b;b_xdd_step];

if any(MPC_inputs.MoS_sampling)==1
     if MPC_inputs.MoS_sampling(1)==1
        b
    end
    [A_MoS,b_MoS]=function_constraint_MoS(MPC_inputs,COM_state_preview);
    A_MoS=[-A_MoS zeros(size(A_MoS,1),size(A,2)-size(A_MoS,2));
        zeros(size(A_MoS,1),size(A_zmp,2)/2) A_MoS zeros(size(A_MoS,1),size(A,2)-size(A_MoS,2)-size(A_zmp,2)/2)];
 
    A=[A;A_MoS];
    b=[b;b_MoS];
   
end

% if any(MPC_inputs.MoS_sampling)==1
%     [A_MoS,b_MoS]=function_constraint_MoSML(MPC_inputs,COM_state_preview);
% %     A_MoS=[A_MoS zeros(size(A_MoS,1),size(A,2)-size(A_MoS,2))];
%     A_MoS=[zeros(size(A_MoS,1),size(A_zmp,2)/2) A_MoS zeros(size(A_MoS,1),size(A,2)-size(A_MoS,2)-size(A_zmp,2)/2)];
%     
% %     A=[A;A_MoS];
% %     b=[b;b_MoS];
% end

dxCoP_max = 0.32; % m/s
[A_xdCOP,b_xdCOP]=function_constraint_dxCoP(CoP_state_preview,dxCoP_max);
A_xdCOP=[A_xdCOP zeros(size(A_xdCOP,1),size(A,2)-size(A_xdCOP,2))];
A=[A;-A_xdCOP];
b=[b;b_xdCOP];

dyCoP_max = 1.45; % m/s
% dyCoP_max = 10; % m/s
[A_ydCOP,b_ydCOP]=function_constraint_dyCoP(CoP_state_preview,dyCoP_max);
A_ydCOP=[zeros(size(A_ydCOP,1),size(A_zmp,2)/2) A_ydCOP zeros(size(A_ydCOP,1),size(A,2)-size(A_ydCOP,2)-size(A_zmp,2)/2)];
A=[A;A_ydCOP];
b=[b;b_ydCOP];

 if MPC_inputs.k<=26
    dyCoP_max = .42; % m/s
    % dyCoP_max = 10; % m/s
    [A_ydCOP,b_ydCOP]=function_constraint_dyCoP(CoP_state_preview,dyCoP_max);
    A_ydCOP=[zeros(size(A_ydCOP,1),size(A_zmp,2)/2) A_ydCOP zeros(size(A_ydCOP,1),size(A,2)-size(A_ydCOP,2)-size(A_zmp,2)/2)];
    A=[A;-A_ydCOP];
    b=[b;b_ydCOP];
end
 

 
% dCoP_max = 1; % m/s
% [A_dCOP,b_dCOP]=function_constraint_dCoP(CoP_state_preview,Step_state_preview,dCoP_max);
% 
% A=[A;A_dCOP];
% b=[b;b_dCOP];

