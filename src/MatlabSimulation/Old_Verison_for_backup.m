%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions For PathPlanning          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function jointList = PathPlanning(initial_joints, TDesired, J_sym, nSteps, link_lengths)
    %allocate the resulting joint list
    jointList = zeros(nSteps, numel(initial_joints));

    %compute the initial solution
    Tcurrent = forward_kinematics(initial_joints, link_lengths);

    %convert the desired transformation matrix to a vector
    des_EulerAngles_XYZ = rotm2eul(TDesired(1:3,1:3),'XYZ')';
    des_Position_XYZ = TDesired(1:3,4);
    cur_EulerAngles_XYZ = rotm2eul(Tcurrent(1:3,1:3),'XYZ')';
    cur_Position_XYZ = Tcurrent(1:3,4);

    desiredVariables = [des_EulerAngles_XYZ; des_Position_XYZ];
    currentVariables = [cur_EulerAngles_XYZ; cur_Position_XYZ];

    %get the step size for each path step
    stepSize = (desiredVariables - currentVariables)/nSteps;

    %set up for the solver
    max_iterations = 15;
    tolerance = 0.001;
    %compute the path
    for i = 1:nSteps
        i
        %compute the desired variables for this step
        currentVariables = currentVariables + stepSize;

        %convert the desired variables to a transformation matrix
        XYZ_desired = currentVariables(4:6);
        euler_desired = currentVariables(1:3)';
        RotM_desired = eul2rotm(euler_desired, 'XYZ');
    
        Tdesired = [ RotM_desired XYZ_desired;
                        0 0 0 1
                    ];
%         Tdesired = eul2tform(currentVariables(1:3)','XYZ') * ([currentVariables(4:6); 1]);

        %do the inverse kinematics 
        [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance);        
        Jacobian = evaluateJacobian(J_sym, desiredJoints)
        det(Jacobian)
        %keep the joint angles between -2pi and 2pi
        for j = 1:numel(desiredJoints)
%             if desiredJoints(j) > 2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), - 2*pi);
%             elseif desiredJoints(j) < -2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), 2*pi);
%             end
              %keep the joint angles between -pi/2 and pi/2
              desiredJoints(j) = max(min(desiredJoints(j), pi), -pi);

        end

        %store the desired joint angles
        jointList(i,:) = desiredJoints;

        %update the initial joints for the next step
        initial_joints = desiredJoints;

    end
end











function jointList = PathPlanning(initial_joints, TDesired, J_sym, nSteps, link_lengths)
    
    %compute the initial solution
    Tcurrent = forward_kinematics(initial_joints, link_lengths);

    %convert the desired transformation matrix to a vector
    des_EulerAngles_XYZ = rotm2eul(TDesired(1:3,1:3),'XYZ')';
    des_Position_XYZ = TDesired(1:3,4);
    cur_EulerAngles_XYZ = rotm2eul(Tcurrent(1:3,1:3),'XYZ')';
    cur_Position_XYZ = Tcurrent(1:3,4);

    desiredVariables = [des_EulerAngles_XYZ; des_Position_XYZ];
    currentVariables = [cur_EulerAngles_XYZ; cur_Position_XYZ];

    %get the step size for each path step
    stepSize = (desiredVariables - currentVariables)/nSteps;

    %set up for the solver
    max_iterations = 15;
    tolerance = 0.001;
    maxSteps = 15;
    %compute the path
    i = 1;
    PoseError = Inf;
    %allocate the resulting joint list
    jointList = zeros(max_iterations, numel(initial_joints));
    while PoseError > tolerance && i < maxSteps
        %Display the current step number
        i
        
        %compute the desired variables for this step
        currentVariables = currentVariables + stepSize;
        %convert the desired variables to a transformation matrix
        XYZ_desired = currentVariables(4:6);
        euler_desired = currentVariables(1:3)';
        RotM_desired = eul2rotm(euler_desired, 'XYZ');
    
        Tdesired = [ RotM_desired XYZ_desired;
                        0 0 0 1
                    ];
        PoseError = norm(compute_pose_error(TDesired,Tdesired));        
        %do the inverse kinematics 
        [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance);        
        
        %Check if we are approaching a singularity
        singular_threshold = 0.01;  % Adjust this value as needed
        Jacobian = evaluateJacobian(J_sym, desiredJoints)
        JacobianDet = det(Jacobian)
        jumpdist = 0.001;
        if (abs(JacobianDet) < singular_threshold)
            currentVariables = currentVariables - 2*stepSize;
%         while(abs(JacobianDet) < singular_threshold)
        if (abs(JacobianDet) < singular_threshold)
            %move the current target position slighty to avoid singulariy
            currentVariables(4:6) = currentVariables(4:6) + [1; 0; 0]*jumpdist
            %do the inverse kinematics again
            [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
            %Check if we are approaching a singularity
            Jacobian = evaluateJacobian(J_sym, desiredJoints)
            JacobianDet = det(Jacobian)
            if (abs(JacobianDet) < singular_threshold)
                %move the current target position back and in the opposite
                %direction
                currentVariables(4:6) = currentVariables(4:6) + [-2; 0; 0]*jumpdist
                %do the inverse kinematics again
                [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
                %Check if we are approaching a singularity
                Jacobian = evaluateJacobian(J_sym, desiredJoints)
                JacobianDet = det(Jacobian)
                if (abs(JacobianDet) < singular_threshold)
                    %move the current target position back and in the opposite
                    %direction
                    currentVariables(4:6) = currentVariables(4:6) + [1; 1; 0]*jumpdist
                    %do the inverse kinematics again
                    [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
                    %Check if we are approaching a singularity
                    Jacobian = evaluateJacobian(J_sym, desiredJoints)
                    JacobianDet = det(Jacobian)
                    if (abs(JacobianDet) < singular_threshold)
                        %move the current target position back and in the opposite
                        %direction
                        currentVariables(4:6) = currentVariables(4:6) + [0; -2; 0]*jumpdist
                        %do the inverse kinematics again
                        [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
                        %Check if we are approaching a singularity
                        Jacobian = evaluateJacobian(J_sym, desiredJoints)
                        JacobianDet = det(Jacobian)
                        if (abs(JacobianDet) < singular_threshold)
                            %move the current target position back and in the opposite
                            %direction
                            currentVariables(4:6) = currentVariables(4:6) + [0; 1; 1]*jumpdist
                            %do the inverse kinematics again
                            [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
                            %Check if we are approaching a singularity
                            Jacobian = evaluateJacobian(J_sym, desiredJoints)
                            JacobianDet = det(Jacobian)
                            if (abs(JacobianDet) < singular_threshold)
                                %move the current target position back and in the opposite
                                %direction
                                currentVariables(4:6) = currentVariables(4:6) + [0; 0; -2]*jumpdist
                                %do the inverse kinematics again
                                [desiredJoints, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance); 
                                %Check if we are approaching a singularity
                                Jacobian = evaluateJacobian(J_sym, desiredJoints)
                                JacobianDet = det(Jacobian)
                                if (abs(JacobianDet) < singular_threshold)
                                    %return to original position
                                    currentVariables(4:6) = currentVariables(4:6) + [0; 0; 1]*jumpdist   
                                    jumpdist = jumpdist*2
                                end
                            end
                        end
                    end
                end
            end
        end
%         end
            
        %now that we have avoided the singularity, recompute the direction
        %to the destination
        newSteps = min(10, maxSteps - i);
        stepSize = (desiredVariables - currentVariables)/newSteps;
        end

        %keep the joint angles between -2pi and 2pi
        for j = 1:numel(desiredJoints)
%             if desiredJoints(j) > 2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), - 2*pi);
%             elseif desiredJoints(j) < -2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), 2*pi);
%             end
              %keep the joint angles between -pi/2 and pi/2
              desiredJoints(j) = max(min(desiredJoints(j), pi), -pi);

        end

        %update the initial joints for the next step
        initial_joints = desiredJoints;
        i = i + 1;
         %store the desired joint angles
        jointList(i,:) = desiredJoints;
        

    end
end