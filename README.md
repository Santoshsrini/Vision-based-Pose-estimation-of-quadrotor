# Vision-based-Pose-estimation-of-quadrotor

 This project presents a vision-based 3D pose and velocity estimator for a quadrotor using AprilTags. The pose estimation leverages camera calibration data, AprilTag corners, and world frame locations to compute the quadrotor's pose for each data packet. Velocity estimation is achieved by extracting corners, computing optical flow, and rejecting outliers using RANSAC. 

The project is divided into 2 parts and the methodology for each is discussed below. 

## Part 1 Methodology:

Part 1 involves estimating the pose of the quadrotor using AprilTags, homography and camera calibration. 

Getting corner cor-ordinates: The first step is to compute the coordinates of the corners in the world frame for which the getCorners.m file was coded. For each detected AprilTag in the dataset at time `t`, the function retrieves the tag's corners in both the world frame and the camera frame, essential for constructing the transformation equations.

AprilTag Processing: For each detected AprilTag in the dataset at time `t`, the function retrieves the tag's corners in both the world frame and the camera frame, essential for constructing the transformation equations.

Matrix Construction: Using the corner coordinates, the function constructs a series of linear equations encapsulated in the matrix `A`. This matrix bridges the spatial information between the camera and world frames.

Singular Value Decomposition (SVD): The function applies SVD on matrix `A` to extract the homography matrix `H`, which represents the transformation from the world frame to the camera frame.

Pose Extraction: Through matrix manipulation involving `H` and the camera intrinsic matrix `k`, the function calculates the rotation matrix `R` and the translation vector `T`, effectively estimating the drone's pose.

 World Frame Transformation: The function then applies additional transformations to convert the pose from the camera frame to the world frame, using the known relationship between the camera and the drone's IMU.

 ## Part 1 Results Dataset 1:

 <img width="1194" alt="image" src="https://github.com/Santoshsrini/Vision-based-Pose-estimation-of-quadrotor/assets/28926309/39519ab8-f03a-4bf7-96ee-d4630e2e9934">
 

 ## Part 1 Results Dataset 4:

<img width="1198" alt="image" src="https://github.com/Santoshsrini/Vision-based-Pose-estimation-of-quadrotor/assets/28926309/c9d4b242-5dd8-4d61-a670-f302e8617398">


## Part 2 Methodology:

Part 2 involves estimating the linear and angular velocity  of the quadrotor using corner detection, optical flow estimation and RANSAC. 

Corner Extraction and Tracking: The corner points are extracted using the FAST Algorithm from MATLAB's Computer Vision Toolbox, with the 100 strongest points being selected for tracking.

Tracking Points in the Second Image: The corner points are tracked into the second image using the KLT tracker. Both the initial corner locations and the tracked points are calibrated to camera coordinates using the camera matrix.

Optical Flow Estimation: The optical flow is determined by the displacement of tracked points from their initial positions. The optical velocity is computed by dividing the optical flow by the time difference (dt), which is refined using a low-pass filter to reduce noise.

Depth Calculation of Corners: The drone's pose is estimated with the `estimatePose` function, providing necessary spatial orientation and position data. Transformation matrices that link the drone's body, camera, and world frames are derived. The depth of each corner is calculated based on geometric relationships established from the pose and camera perspective.

Optional RANSAC for Velocity Estimation: If enabled, the velocity of the camera relative to the world frame is robustly estimated using the `velocityRANSAC` function. In the absence of RANSAC, velocity is calculated directly using all points, providing a straightforward method that may be more susceptible to outliers.

Velocity Storage: The calculated velocity is stored in the `estimatedV` variable, conforming to a specified structure that details linear and angular velocity components.





 
