function theta = inverse_kine_CCD(desired_traj, robot)
    [M, N] = size(desired_traj);
    theta = zeros(M, 4);
    
    %For each trajactory point 
    for i = 1:M
        x = desired_traj(i,1);
        y = desired_traj(i,2);
        % The theta for initial state is [0,0,0,0];The rest is previous
        % theta
        if i == 1
            curr_theta = zeros(1,4);
            %curr_theta = [1.5437,0.0237,-0.0018,-0.0034];
        else
            curr_theta = theta(i-1,:);
        end
        %curr_theta = [1.5681,0.0307,0.0051,-0.1370];
        
        counter = 0;
        %Loop until the error is satisfied.
        while true
            %Find the current end-factor position.
            t_matrix = apply_trans(curr_theta,4);
            end_x = t_matrix(1,4);
            end_y = t_matrix(2,4); 
            
            %Calculate the error based on the t_matrix(L2 Norm)
            delta_x = abs(end_x - x);
            delta_y = abs(end_y - y);
            
            %error and the distance between end factor and target
            %The error is also the distance between target and end-factor
            error = (delta_x^2) + (delta_y^2);
            error = sqrt(error);
            
            %Set the desired error, if the error is less, the jump out
            accepted_error = 0.472;
            if(error <= accepted_error)
                break;
            end
            
            %For each joint 4,3,2,1
            joint_index = mod(counter,4) + 1;
            
            %Work out the joint position, the first joint is 0,0 for ever.
            if(joint_index == 1)
                joint_x = 0;
                joint_y = 0;
            else
                temp_matrix = apply_trans(curr_theta,joint_index-1);
                joint_x = temp_matrix(1,4);
                joint_y = temp_matrix(2,4);
            end
            
            %The distance between joint and end factor
            join2end = sqrt((joint_x - end_x)^2 + (joint_y - end_y)^2);
            %The distance between target and joint
            tar2join = sqrt((x - joint_x)^2 + (y - joint_y)^2);
            
            %Calculate the delta theta(degree) by using the Law of cosines
            cos_theta = (tar2join^2 + join2end^2 - error^2) / (2 * tar2join * join2end);
            delta_theta = acos(cos_theta)/180*pi;
            
            %Add or substract the theta is based on 
            slope = (y - joint_y)/(x - joint_x);         
            bias = y - slope * x;
            line_y = end_x * slope + bias;
            sign = ((line_y - end_y)/abs((end_y -line_y))) * (slope/abs(slope));
            
            %Update the joint theta
            curr_theta(joint_index) = curr_theta(joint_index) + (sign) * delta_theta;
            for temp_1 = 1:4
                recorder(counter+1,temp_1) = curr_theta(temp_1);
            end 
            
            if counter > 10000
                break;
            end
            
            %counter start from 1
            counter = counter + 1;
        end
        
        
%           figure(1)
%           plot3(desired_traj(:, 1), desired_traj(:, 2), zeros(315, 1));
%           view(0, 90);
%           hold on
%           robot.plot(recorder, 'trail', 'r-')
%           hold off

        
        for temp_1 = 1:4
            theta(i,temp_1) = curr_theta(temp_1);
        end        
    end
end

%Build the transformation matrix based on the theta
function matrix = apply_trans (theta_row,index)       
    theta = theta_row(1);
    matrix = [cos(theta),-sin(theta),0,cos(theta);
              sin(theta),cos(theta) ,0,sin(theta);
              0         ,0          ,1,         0;
              0         ,0          ,0,         1];
    
    if index == 1
        return
    end
    
    for j = 2:(index)
        theta = theta_row(j);
        matrix2 = [cos(theta),-sin(theta),0,cos(theta);sin(theta),cos(theta),0,sin(theta);0,0,1,0;0,0,0,1];
        matrix = matrix * matrix2;
    end
end
    