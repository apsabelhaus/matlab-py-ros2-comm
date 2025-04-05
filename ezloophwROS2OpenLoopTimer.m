function ezloophwROS2OpenLoopTimer(~, ~, handles)
    %ezloophwROS2OpenLoopTimer - Timer update function that sends open loop
    %signals per a passed-in controller function. Starts at t=0 when the
    %timer is created.
    % (C) Soft Robotics Control Lab, 2025  

    % Update the control message values
    if isvalid(handles.controlPub)
        % net time since start, in seconds
        tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
        t = tnow - handles.timeatstart;
        % the open loop command
        u_ol = handles.ctrlr(t, handles.ctrl_param);
        handles.controlPubmsg.data = u_ol;

        % handles.controlPubmsg.data = rand(2,1);
        % Negative and positive numbers between [-2000, 2000]
        %handles.controlPubmsg.data = 2000*rand(2,1) - [2000; 2000];
        % for the pneumatics hardware, really is +/- 200 or so.
        % handles.controlPubmsg.data = 200*rand(2,1) - [100; 100];
        disp(handles.controlPubmsg.data)
        % Publish the control input message
        send(handles.controlPub, handles.controlPubmsg);
    end
end