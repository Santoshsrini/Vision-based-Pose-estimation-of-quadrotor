function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter 


    % Probability of success
    P_success = 0.925;

    % Minimum number of points
    M = 3; 

    % Threshold
    beta = 0.1;

    % Calculate the number of probabilistic iterations needed
    iterations = ceil(log(1 - P_success) / log(1 - e^(M)));

    % Get the total number of points chosen in the image
    total_num_points = size(optPos, 1);

    % Initialize the maximum inlier count
    max_inlier_count = 0;

    %disp(iterations);

    % Main RANSAC loop
    for i=1:iterations

        % Randomly select M points and extract their corresponding
        % position, optical velocity and depth from drone
        selected_points_indices = randperm(total_num_points, M);
        selected_points_vel = optV(selected_points_indices,:);
        selected_points_pos = optPos(selected_points_indices,:);
        selected_points_depth = Z(selected_points_indices);

        % p1 is point 1 in selected point, p2 is point2 and so
        % p1 is (x1,y1) and p2 is (x2, y2)
        % just to make writing H easier
        p1 = selected_points_pos(1,:);
        p2 = selected_points_pos(2,:);
        p3 = selected_points_pos(3,:);

        % z1 is depth of point 1 and z2 is depth of point 2 and so on
        z1 = selected_points_depth(1);
        z2 = selected_points_depth(2);
        z3 = selected_points_depth(3);

        % for the chosen 3 points construct the H matrix 
          
        H_opt = [-1/z1, 0, p1(1)/z1, p1(1)*p1(2), -(1+p1(1)^2), p1(2);
                0, -1/z1, p1(2)/z1, (1+p1(2)^2), -p1(1)*p1(2), -p1(1);
                -1/z2, 0, p2(1)/z2, p2(1)*p2(2), -(1+p2(1)^2), p2(2);
                0, -1/z2, p2(2)/z2, (1+p2(2)^2), -p2(1)*p2(2), -p2(1);
                -1/z3, 0, p3(1)/z3, p3(1)*p3(2), -(1+p3(1)^2), p3(2);
                0, -1/z3, p3(2)/z3, (1+p3(2)^2), -p3(1)*p3(2), -p3(1)];

        % reshaping the optical flow velocity of the selected 3 points
        selected_points_vel_column = reshape(selected_points_vel', [], 1);

        % compute the camera velocity using H and optical flow vel
        optimal_camera_velocity = H_opt\selected_points_vel_column;

        % with this computed camera velocity we begin to estimate the inliers
        % once this is done we use all the inliers to compute the final
        % optimal camera velocity

        % Initialize inlier tracking
        inlier_indices = -ones(1, total_num_points);
        inlier_count = 0;

        % Loop over all points to find inliers
        for j=1:total_num_points

            % Construct the H_j matrix for each point
            H_j = [-1/Z(j), 0, optPos(j,1)/Z(j), (optPos(j,1)*optPos(j,2)), -(1+optPos(j,1)^2), optPos(j,2);
                0, -1/Z(j), optPos(j,2)/Z(j), (1+optPos(j,2)^2), -optPos(j,1)*optPos(j,2), -optPos(j,1)];


            % Predict the velocity and compare with optical flow velocity
            % to get the error
            pred_vel = H_j*optimal_camera_velocity; 
            p_dot = optV(j,:)';
            error = norm(pred_vel - p_dot);
            
            % update the inlier count based on threshold error and store
            % the index of all the inliers
            if error < beta
                inlier_count=inlier_count+1;
                inlier_indices(inlier_count) = j;
            end

        end
        
        % formatting to get the final list of all inlier indices
        inlier_indices = inlier_indices(1:inlier_count);


        % Update the best model if the current one has more inliers
        if inlier_count>max_inlier_count

            max_inlier_count = inlier_count;

            % Extract all the inlier positions and velocities
            inlierPos = optPos(inlier_indices, :);
            inlierVel = optV(inlier_indices,:);
            inlierDepth = Z(inlier_indices);

            % Construct the H matrix for all inliers
            H = [];
            for k=1:inlier_count
                x = inlierPos(k,1);
                y = inlierPos(k,2);
    
                H_temp = [-1/inlierDepth(k), 0, x/inlierDepth(k), (x*y), -(1+x^2), y;
                    0, -1/inlierDepth(k), y/inlierDepth(k), (1+y^2), -x*y, -x];
    
                % Vertically stack all A matrices
                H = vertcat(H,H_temp); 

            end
            
            % reshaping the inlier velocity vector
            inlierVel_column = reshape(inlierVel', [], 1);

            % Compute the final velocity in the camera frame using all the
            % inliers
            final_velocity_camera = pinv(H)*inlierVel_column;

            % Transform the velocity into the world frame
            z = zeros(3);
            new_matrix = [R_c2w z; z R_c2w];
            velocity_camera_in_world = new_matrix*final_velocity_camera;
            Vel = velocity_camera_in_world;


        end

    end


    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end