function theta = inverse_kine_closedform(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    if (strcmp(option, 'q4'))
        theta = zeros(M, 5);
        %For each desired points
        for i=1:M
            desired_x = desired_traj(i,1);
            desired_y = desired_traj(i,2);
            desired_z = desired_traj(i,3);
            desired_r = desired_traj(i,4:6);
            
            %Theta 1
            theta(i,1) = atan2(desired_y,desired_x);
            
            %Work out the transformation matrix
            rot_matrix = rodrigues(desired_r);
            T_0_5 = cat(2,rot_matrix,[desired_x;desired_y;desired_z]);
            T_0_5 = cat(1,T_0_5,[0,0,0,1]);
            
            %theta 5
            R31 = rot_matrix(3,1);
            R32 = rot_matrix(3,2);
            theta(i,5) = atan2(R32,-R31);
            
            %Transformation matrix from 1 to 4
            T_0_1 = robot.A(1,theta(i,1));
            T_4_5 = robot.A(5,theta(i,:));
            T_1_4 = inv(T_0_1) * T_0_5 * inv(T_4_5);
            
            %Theta 3
            r14 = T_1_4(1,4);
            r24 = T_1_4(2,4);
            a2 = 0.155;
            a3 = 0.135;
            theta(i,3) = acos((r14^2 + r24^2 - a2^2 - a3^2)/(2*a2*a3));
            
            %Theta 2
            theta(i,2) = atan2(-r14,r24) - atan2(a3*sin(theta(i,3)),a2+a3*cos(theta(i,3)));
            
            %Theta 4 
            r11 = T_1_4(1,1);
            r21 = T_1_4(2,1);
            theta(i,4) = atan2(r21,r11) - theta(i,2) - theta(i,3);
        end
    end
end