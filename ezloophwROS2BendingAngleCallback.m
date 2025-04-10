function ezloophwROS2BendingAngleCallback(message)
    %exampleHelperROS2PoseCallback Subscriber callback function for pose data    
    %   exampleHelperROS2PoseCallback(MESSAGE) returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS 2 message MESSAGE.
    %   
    %   See also ROSPublishAndSubscribeExample.
    
    %   Copyright 2019 The MathWorks, Inc.
    
    % Declare global variables to store bending angle
    global bendingVec
    
    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.

    if ~isempty(message.data)
        bendingVec = message.data(:)';  % Row vector
    disp("Received " + string(bendingVec));
    end

end