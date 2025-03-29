function ezloophwROS2SimTimer(~, ~, handles)
    %exampleHelperROS2SimTimer - Timer update function called in startExamples to publish
    %   messages at well-defined intervals
    %   exampleHelperROS2SimTimer(~,~,HANDLES) publishes /pose and /scan messages at
    %   regular intervals. The /scan message value remains constant while
    %   the /pose message value changes continually.
    %   
    %   See also exampleHelperROS2CreateSampleNetwork
    
    %   Copyright 2019 The MathWorks, Inc.
    
    % Update the pose message values
    if isvalid(handles.controlPub)
        handles.controlPubmsg.data = rand(2,1);
        % disp('callback pub')
        % disp(handles.controlPubmsg.Data);
        % handles.controlPubmsg.linear.x = handles.controlPubmsg.linear.x + (rand(1)-0.5)./10;
        % handles.controlPubmsg.linear.y = handles.controlPubmsg.linear.y + (rand(1)-0.5)./10;
        % handles.controlPubmsg.linear.z = handles.controlPubmsg.linear.z + (rand(1)-0.5)./10;
        % handles.controlPubmsg.angular.x = handles.controlPubmsg.angular.x + (rand(1)-0.5)./10;
        % handles.controlPubmsg.angular.y = handles.controlPubmsg.angular.y + (rand(1)-0.5)./10;
        % handles.controlPubmsg.angular.z = handles.controlPubmsg.angular.z + (rand(1)-0.5)./10;
        % 
        % Publish the scan and pose messages
        send(handles.controlPub, handles.controlPubmsg);
    end
end