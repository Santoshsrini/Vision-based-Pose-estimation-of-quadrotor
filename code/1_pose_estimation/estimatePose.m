function [position, orientation] = estimatePose(data, t)
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
    
    % Initialize matrix A
    A = [];

    % Camera Matrix (zero-indexed):
    k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];

    % iterating over all April Tags in the image
    for i=1:length(data(t).id)
        % Get the corners of each April Tag in the world frame
        pw = getCorner(data(t).id(i));
        % Get corners of each April Tag in the camera frame
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
    
    % perform svd of A 
    [U, S, V] = svd(A);
   
    % Extract the last column of V, reshape it, and adjust sign based on the last element
    H = sign(V(9,9)) * [V(1:3,9), V(4:6,9), V(7:9,9)]';

    % Decompose the homography matrix to extract rotation
    R = k\H;

    % Normalize the rotation matrix
    R1 = R(:,1);
    R2 = R(:,2);
    T = R(:,3);
    
    % Compute the SVD of the newly formed orthogonal matrix
    [U, ~, V] = svd([R1, R2, cross(R1,R2)]);

   % Re-orthogonalize to ensure R is a valid rotation matrix
    R = U * [1, 0, 0; 0, 1, 0; 0, 0, det(U*V')] * V';
    
    % Scale T
    T = T / norm(R1) ;


    % Combine rotational and translational parts from world to camera 
    Tw_c = [[R, T] ; [0,0,0,1]];
    
    % Find the Homogenous transform to go from imu frame to camera frame
    Rc_b = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1]; % eul2rotm([-pi/4,pi,0])
    Tc_b = [0.0283; -0.0283; 0.0300]; % eul2rotm([-pi/4,pi,0]) * [-0.04, 0.0, -0.03]';
    Tb_c = inv([Rc_b, Tc_b; [0,0,0,1]]);
    
    % Transform from imu frame to world
    HT = Tw_c\Tb_c;
    
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX

    % Set the position
    % position of drone in world
    position = HT(1:3, 4);

    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % Set the orientation
    % orientation of drone in the world
    orientation = rotm2eul(HT(1:3, 1:3)) ;
    
end