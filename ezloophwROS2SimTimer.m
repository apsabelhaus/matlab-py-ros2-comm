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
        % handles.controlPubmsg.data = rand(2,1);
        % Negative and positive numbers between [-2000, 2000]
        %handles.controlPubmsg.data = 2000*rand(2,1) - [2000; 2000];
        % for the pneumatics hardware, really is +/- 200 or so.
        handles.controlPubmsg.data = 200*rand(2,1) - [100; 100];
        disp(handles.controlPubmsg.data)
        % Publish the control input message
        send(handles.controlPub, handles.controlPubmsg);
    end
end