function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset

    
    % Initialize the matrix A for stacking all transformation matrices
    A = [];

    % Define the camera intrinsic matrix
    k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];

    % Loop over all detected AprilTags
    for i=1:length(data(t).id)

        % Extract world frame corner coordinates for the current tag
        pw = getCorner(data(t).id(i));
        
        % Extract image frame corner coordinates for the current tag
        pc = [data(t).p4(:,i),data(t).p3(:,i),data(t).p2(:,i),data(t).p1(:,i)];
      
        % Find the A matrix 
        A_temp = [   pw(1,1), pw(2,1), 1, 0, 0, 0, (-pc(1,1)*pw(1,1)), (-pc(1,1)*pw(2,1)), (-pc(1,1));
            0, 0, 0, pw(1,1), pw(2,1), 1, (-pc(2,1)*pw(1,1)), (-pc(2,1)*pw(2,1)), (-pc(2,1));
            pw(3,1), pw(4,1), 1, 0, 0, 0, (-pc(1,2)*pw(3,1)), (-pc(1,2)*pw(4,1)), (-pc(1,2));
            0, 0, 0, pw(3,1), pw(4,1), 1, (-pc(2,2)*pw(3,1)), (-pc(2,2)*pw(4,1)), (-pc(2,2));
            pw(5,1), pw(6,1), 1, 0, 0, 0, (-pc(1,3)*pw(5,1)), (-pc(1,3)*pw(6,1)), (-pc(1,3));
            0, 0, 0, pw(5,1), pw(6,1), 1, (-pc(2,3)*pw(5,1)), (-pc(2,3)*pw(6,1)), (-pc(2,3));
            pw(7,1), pw(8,1), 1, 0, 0, 0, (-pc(1,4)*pw(7,1)), (-pc(1,4)*pw(8,1)), (-pc(1,4));
            0, 0, 0, pw(7,1), pw(8,1), 1, (-pc(2,4)*pw(7,1)), (-pc(2,4)*pw(8,1)), (-pc(2,4));];
        % Vertically stack all A matrices
        A = vertcat(A,A_temp); 
    end

     % Perform Singular Value Decomposition (SVD) on the accumulated matrix A
    [U, S, V] = svd(A);
   
    % Extract the homography matrix from the last column of V
    H = sign(V(9,9)) * [V(1:3,9), V(4:6,9), V(7:9,9)]';


    % Decompose the homography matrix to extract rotation and translation
    R = k\H;
    R1 = R(:,1);
    R2 = R(:,2);
    T = R(:,3);
    
    % Compute the SVD of the newly formed orthogonal matrix
    [U, ~, V] = svd([R1, R2, cross(R1,R2)]);

    % Reconstruct the orthogonal rotation matrix 
    R = U * [1, 0, 0; 0, 1, 0; 0, 0, det(U*V')] * V';
    
    % Scale T
    T = T / norm(R1) ;


    % Combine rotational and translational parts to get transformation from world to camera 
    Tw_c = [[R, T] ; [0,0,0,1]];
    
    % Find the Homogenous transform to go from imu frame to camera frame
    Rc_b = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1]; % eul2rotm([-pi/4,pi,0])
    Tc_b = [0.0283; -0.0283; 0.0300]; % eul2rotm([-pi/4,pi,0]) * [-0.04, 0.0, -0.03]';
    Tb_c = inv([Rc_b, Tc_b; [0,0,0,1]]);
    
    % Transformation from imu frame to world
    HT = Tw_c\Tb_c;
    
  

    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX

    % Set the position
    position = HT(1:3, 4);
    
    % Set the orientation
    orientation = rotm2eul(HT(1:3, 1:3)) ;
    
    %R_c2w = Rotation which defines camera to world frame
    R_c2w = R';
end