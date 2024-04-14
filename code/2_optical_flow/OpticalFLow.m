%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION


% camera calibration matrix
k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];

% probability of one point being an inlier
e=0.7;


% ransac flag to toggle in using RANSAC mode or not
ransac_flag = 0;

% Apply filter to smooth the timestamp data
t = sgolayfilt([sampledData.t],1,101);


% Main loop to process each data 
for n = 2:length(sampledData)
    %% Initalize Loop load images

    % Load consecutive frames for processing

    img1 = sampledData(n-1).img;
    img2 = sampledData(n).img; 

     disp(n);

     % Calculate time difference between frames
     dt = t(n) - t(n-1);
    
    %% Detect good points


    % Detect corner points using the FAST algorithm
    corners = detectFASTFeatures(img1, 'MinContrast', 0.1);


    % Select the strongest corner points
    num_strongest = 100; % Specify the number of strongest points to select
    strongest_corners = selectStrongest(corners, num_strongest);

    % Extract the pixel locations of the selected corners
    cornerLocations = strongest_corners.Location;


    
    %% Initalize the tracker to the last frame.
    
    % Create a KLT tracker object

    tracker = vision.PointTracker('MaxBidirectionalError', 1);

    % Initialize the tracker with the corner points from the first image

    initialize(tracker, cornerLocations, img1);


    %% Find the location of the next points;

    % Track the points in the second image

    [points, validity] = tracker(img2);

    % Calibrate the corner locations to camera coordinates
    calibrated_cornerLocations = k \ [cornerLocations, ones(num_strongest,1)]';
    calibrated_cornerLocations = [calibrated_cornerLocations(1,:); calibrated_cornerLocations(2,:)]';
    cornerLocations = calibrated_cornerLocations; 

    % Calibrate the tracked points to camera coordinates 
    calibrated_points = k \ [points, ones(num_strongest,1)]';
    calibrated_points = [calibrated_points(1,:); calibrated_points(2,:)]';
    points = calibrated_points; 


    
    % Estimate the optical flow applying delta x and delta y between corner
    % locations in the first image and tracked points in the second
    flow = points - cornerLocations;
    
    % Ensure the validity of optical flow vectors and corners points
    % basically checking if the corresponding points are present in both
    % frames
    validFlow = flow(validity, :);
    valid_corner = cornerLocations(validity,:);
    
    %% Calculate velocity
    % Use a for loop

    % Velocity is computed as the flow divided by the time difference
    flow_velocity = validFlow/dt;
    
    %% Calculate Height

    % Initialize a vector for storing depth of all corners
    Z = zeros(length(flow_velocity), 1);

    % Use estimated pose to get the drone's position, orientation, and rotation from camera to world frame
    [position, orientation, R_c2w] = estimatePose(sampledData, n);

    % Transformation matrices are computed between each of body, camera and world
    % frame are computed to estimate the depth of each corner
    Rb_w = eul2rotm(orientation);
    Tb_w = [[Rb_w, position] ; [0,0,0,1]];

    
    Rc_b = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1]; % eul2rotm([-pi/4,pi,0])
    pc_b = [0.0283; -0.0283; 0.0300]; % eul2rotm([-pi/4,pi,0]) * [-0.04, 0.0, -0.03]';
    Tc_b = [[Rc_b, pc_b]; [0,0,0,1]];
    
    Tc_w = Tb_w*Tc_b;
    Rc_w = Tc_w(1:3,1:3); 
    
    Tw_c = inv(Tc_w);
    Rw_c = Tw_c(1:3, 1:3);
    pw_c = Tw_c(1:3,4);


    % estimation the depth of each corner using the formula in the slides
    % 0 = lambda * (Rc2w)*inv(camera-matrix)*(Xc; Yc; 1) - (Rc2w)*(p_w2c)
    % lambda = Zc is computed from this expression, check report for the
    % math

    for i=1:length(Z)

        % Select the i-th row corner_location
        selected_row = valid_corner(i,:);
        
        % Take the transpose and append 1 to get [Xc;Yc;1]
        image_vector = [selected_row'; 1];
    
        A1 = Rc_w * pw_c;
        B1 = Rc_w * (k\image_vector);

        % estimate depth of each corner from camera
        Z(i) = A1(3,1)/B1(3,1);
    end
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC

    
    % Depending on flag use RANSAC or not

    % for no RANSAC
    if ransac_flag == 0

        H = [];
        % use all the points for computing H and subsequently camera
        % velocity
        for i=1:length(valid_corner)

            x = valid_corner(i,1);
            y = valid_corner(i,2);

            H_temp = [-1/Z(i), 0, x/Z(i), (x*y), -(1+x^2), y;
                0, -1/Z(i), y/Z(i), (1+y^2), -x*y, -x];

            % Vertically stack all  matrices
            H = vertcat(H,H_temp); 

        end

        % reshaping optical flow velocity
        flow_velocity_column = reshape(flow_velocity', [], 1);

        % computing camera velocity in camera frame 
        velocity_camera = pinv(H)*flow_velocity_column;

        % Transform the velocity into the world frame
        z = zeros(3);    
        new_matrix = [Rc_w z; z Rc_w];
        velocity_camera_in_world = new_matrix*velocity_camera;
    
        Vel = velocity_camera_in_world;

    elseif ransac_flag == 1

        % if RANSAC mode is used calling the velocityRANSAC function 
        % directly to compute Vel of camera in world frame

        Vel = velocityRANSAC(flow_velocity,valid_corner,Z,Rc_w,e);

    end

       









    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame


 
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
