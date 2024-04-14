function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)

    % Initialize result matrix to hold coordinates of corners and center for each id
    res = zeros(10,length(id)); 


    for i = 1:length(id)
        % Extract the current ID from the list
        current_id = id(i);

        % Calculate row index and column index based on the current ID
        row = mod(current_id, 12); 
        column = floor(current_id / 12); 


        % Calculate x coordinate based on row index
        x = 0.152*row*2;
       
        % Calculate y coordinate based on column index
        if column <= 2
            y = 0.152*2*column;
        elseif column <= 5
            y = 0.152*2*2 + 0.152 + 0.178 + (column-3)*0.152*2;
        elseif column <= 8
            y = 0.152*2*2 + 0.152 + 0.178 + 0.152*2*2 + 0.152 + 0.178 + (column-6)*0.152*2;
        end

        % Calculate coordinates for each corner and center based on x and y
        p4 = [x; y];
        p3 = [x; y + 0.152];
        p2 = [x + 0.152; y + 0.152];
        p1 = [x + 0.152; y];
        p0 = (p4+p2)/2;


        % Append p4,p3,p2,p1 in the columns of res
        % every column has the corner and center of April tag
        res(:,i) = [p4;p3;p2;p1;p0];

    end
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end