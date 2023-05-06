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