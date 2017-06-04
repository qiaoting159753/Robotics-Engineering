function theta = inverse_kine_jacobian(desired_traj, robot, option)
    if (strcmp(option, 'q1'))
        [M, N] = size(desired_traj);
        theta = zeros(M, 4);
    
        %Foe each desired point
        for i = 1:M
            x_temp = desired_traj(i,1);
            y_temp = desired_traj(i,2);
            
            length = robot.links(1).a + robot.links(2).a + robot.links(3).a + robot.links(4).a; 
            ratio = length/sqrt(x_temp^2 + y_temp^2);
            x = x_temp * ratio;
            y = y_temp * ratio;
        
            % The theta for initial state is [0,0,0,0];The rest is previous
            % theta
            if i == 1
                cthe = zeros(1,4);
            else
                cthe = theta(i-1,:);
            end

            %Loop until converge.
            while true 
                %Apply current theta
                t_matrix = robot.fkine(cthe);
                end_x = t_matrix(1,4);
                end_y = t_matrix(2,4);
            
                %Check converge or not
                %Calculate the error based on the t_matrix(L2 Norm)
                delta_x = abs(end_x - x);
                delta_y = abs(end_y - y);
            
                %error and the distance between end factor and target
                %The error is also the distance between target and end-factor
                error = (delta_x^2) + (delta_y^2);
                error = sqrt(error);
            
                %Set the desired error, if the error is less, the jump out
                accepted_error = 0.05;
                if(error <= accepted_error)
                    break;
                end
            
                t_1234 = sum(cthe);
                t_123 = cthe(1) + cthe(2) + cthe(3);
                t_12 = cthe(1) + cthe(2);
     
                %Calculate Jacobian matrix
                %x,y respect to theta 1
                dev_1_x = -sin(cthe(1))-sin(t_12)-sin(t_123)-sin(t_1234);
                dev_1_y =  cos(cthe(1))+cos(t_12)+cos(t_123)+cos(t_1234);
            
                %x,y respect to theta 2
                dev_2_x =  -sin(t_12)-sin(t_123)-sin(t_1234);
                dev_2_y =  cos(t_12)+cos(t_123)+cos(t_1234);
            
                %x,y respect to theta 3            
                dev_3_x =  -sin(t_123)-sin(t_1234);
                dev_3_y =  cos(t_123)+cos(t_1234);
            
                %x,y respect to theta 4            
                dev_4_x =  -sin(t_1234);
                dev_4_y =  cos(t_1234);
            
                %2X4 matrix
                jacobian = [dev_1_x,dev_2_x,dev_3_x,dev_4_x;
                        dev_1_y,dev_2_y,dev_3_y,dev_4_y];
            
                %4X2 matrix
                inv_jaco = pinv(jacobian);
                    
                %Compute the pose
                pose = [end_x,end_y];
                desired_pose = [x,y];
                pose_delta = desired_pose - pose;
            
                %Update the Jacobian
                delta = (inv_jaco * pose_delta')';
                cthe = cthe + mod(delta*pi/180,2*pi);
            end
       
            for temp_1 = 1:4
                theta(i,temp_1) = mod(cthe(temp_1),2*pi);
            end
        end
        
    elseif (strcmp(option, 'q4'))
        [M, N] = size(desired_traj);
        theta = zeros(M, 5);
    
        %Foe each desired point
        for i = 1:M
            x_temp = desired_traj(i,1);
            y_temp = desired_traj(i,2);
            z_temp = desired_traj(i,3);
            
            x = x_temp;
            y = y_temp;
            z = z_temp;

            % The theta for initial state is [0,0,0,0];The rest is previous
            % theta
            if i == 1
                cthe = zeros(1,5);
            else
                cthe = theta(i-1,:);
            end

            %Loop until converge.
            while true 
                %Apply current theta
                t_matrix = robot.fkine(cthe);
                end_x = t_matrix(1,4);
                end_y = t_matrix(2,4);
                end_z = t_matrix(3,4);
                %Check converge or not
                %Calculate the error based on the t_matrix(L2 Norm)
                delta_x = abs(end_x - x);
                delta_y = abs(end_y - y);
                delta_z = abs(end_z - z);
                %error and the distance between end factor and target
                %The error is also the distance between target and end-factor
                error = (delta_x^2) + (delta_y^2) + (delta_z^2);
                error = sqrt(error);
            
                %Set the desired error, if the error is less, the jump out
                accepted_error = 0.05;
                if(error <= accepted_error)
                    break;
                end

                t_23 = cthe(2) + cthe(3);
                t_234 = cthe(2) + cthe(3) +cthe(4);
                
                %Calculate Jacobian matrix
                %Column 1
                dev_1_1 = 0.218*sin(cthe(1))*sin(t_234) + 0.135*sin(cthe(1))*sin(t_23) + 0.155*sin(cthe(1))*sin(cthe(2));
                dev_2_1 =-0.218*cos(cthe(1))*sin(t_234) - 0.135*cos(cthe(1))*sin(t_23) - 0.155*cos(cthe(1))*sin(cthe(2));
                
                %Column 2
                dev_1_2 =-0.218*cos(cthe(1))*cos(t_234) - 0.135*cos(cthe(1))*cos(t_23) - 0.155*cos(cthe(1))*cos(cthe(2));
                dev_2_2 =-0.218*sin(cthe(1))*cos(t_234) - 0.135*sin(cthe(1))*cos(t_23) - 0.155*sin(cthe(1))*cos(cthe(2));
                dev_3_2 =-0.218*sin(t_234) - 0.135*sin(t_23) - 0.155*sin(cthe(2));
                dev_4_2 = sin(cthe(1));
                dev_5_2 = -cos(cthe(1));
                
                %Column 3
                dev_1_3 =-0.218*cos(cthe(1))*cos(t_234)-0.135*cos(cthe(1))*cos(t_23);
                dev_2_3 =-0.218*sin(cthe(1))*cos(t_234)-0.135*sin(cthe(1))*cos(t_23);
                dev_3_3 =-0.218*sin(t_234)-0.135*sin(t_23);
                dev_4_3 = sin(cthe(1));
                dev_5_3 = -cos(cthe(2));
            
                %Column 4          
                dev_1_4 = -0.218*cos(cthe(1))*cos(t_234);
                dev_2_4 = -0.218*sin(cthe(1))*cos(t_234);
                dev_3_4 = -0.218*sin(t_234);
                dev_4_4 = sin(cthe(1));
                dev_5_4 = -cos(cthe(1));
            
                %Column 5          
                dev_4_5 =  -cos(cthe(1))*sin(t_234);
                dev_5_5 =  -sin(cthe(1))*sin(t_234);
                dev_6_5 = cos(t_234);
            
               
                jacobian = [dev_1_1,dev_1_2,dev_1_3,dev_1_4,0;
                            dev_2_1,dev_2_2,dev_2_3,dev_2_4,0;
                                0  ,dev_3_2,dev_3_3,dev_3_4,0;
                                0  ,dev_4_2,dev_4_3,dev_4_4,dev_4_5;
                                0  ,dev_5_2,dev_5_3,dev_5_4,dev_5_5;
                                1  ,    0  ,    0  ,    0  ,dev_6_5];
            
                %Inverse Jacobian
                inv_jaco = pinv(jacobian(1:3,:));
                    
                %Compute the pose
                pose = [end_x,end_y,end_z];     
                desired_pose = desired_traj(i,1:3);
                pose_delta = desired_pose - pose;
                
                %Update the Jacobian
                delta = (inv_jaco * pose_delta')';
                cthe = cthe + mod(delta*pi/180,2*pi);
            end

            for temp_1 = 1:5
                theta(i,temp_1) = mod(cthe(temp_1),2*pi);
            end
        end
    elseif (strcmp(option, 'qextra'))
        %Damped Least square
        %theta = zeros(M, 7);
    end %End of string compare
end %End of function