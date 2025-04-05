% ros2_matlab_openloop.m
% (C) Soft Robotics Control Lab 2025
% Open loop sequence of control inputs sent over ros.

%%% Setup just to make sure we start at the same state each time.
clear all
close all
clc;

disp('ROS2 Matlab Open Loop Control for ezloophw.');
disp('(C) Soft Robotics Control Lab at Boston University, 2025');
disp('creating the nodes / starting ros2...');

%%% Initialize ROS2 nodes for the MATLAB side
% we are this node for sending control commands
matlabPubNodeName = "matlab_cmder";
matlabPubNode = ros2node(matlabPubNodeName);
% we are this node for receiving joint angles
matlabSubNodeName = "matlab_receiver";
matlabSubNode = ros2node(matlabSubNodeName);

% --- Setup: Publisher to controller commands topic ---

pubWait = 2;
cmdtopic = '/cmd_u_t';

disp("Starting the publisher, waiting " + string(pubWait) + " seconds...");

controlPub = ros2publisher(matlabPubNode, cmdtopic, 'std_msgs/Float64MultiArray');
controlMsg = ros2message(controlPub);
pause(pubWait); %wait for some time to register publisher on the network

% --- Setup: Subscriber to /bending_angles ---

subStarted = 0;
substartwait = 5;
subtopic = 'ezloophw_sensors';

% currently unused, but this variable would contain the data received from
% ros
global bendingVec;

disp('Attempting to start the subscriber...')
while ~subStarted
    try
        bendingSub = ros2subscriber(matlabSubNode, subtopic, @ezloophwROS2BendingAngleCallback);
        subStarted = 1;
    catch
        disp("ERROR! You need to start the publisher in python for topic:");
        disp(subtopic)
        disp("Waiting another " + string(substartwait) + " seconds, trying again...");
        pause(substartwait);
    end
end

% --- Rates ---
% pubRate = 1;    % 1 Hz publish
% pubRate = 5.0;    % this is period not frequency
pubRate = 0.05;     % this is half one control period, dt=0.1 sec

% --- Initialize timing ---

%%%% For the publisher:
% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of pubRate.
timerHandles.controlPub = controlPub;
timerHandles.controlPubmsg = controlMsg;
timerHandles.node = matlabPubNode;

% parameters for the open loop controller.
ctrl_param.amp1 = 100;
ctrl_param.amp2 = 100;
ctrl_param.per1 = 30;
ctrl_param.per2 = 40;
ctrl_param.shift1 = 0;
ctrl_param.shift2 = 0;
ctrlr = @u_openloopsine;
timerHandles.ctrl_param = ctrl_param;
timerHandles.ctrlr = ctrlr;

% timerHandles.timeatstart = ros2time(matlabPubNode, 'now');
timerHandles.timeatstart = double(ros2time(matlabPubNode,'now').sec) + double(ros2time(matlabPubNode,'now').nanosec) * 1e-9;
simTimer = ezloophwROS2Timer(pubRate, {@ezloophwROS2OpenLoopTimer,timerHandles});

%%% Main loop. Just waits until user clicks q in window,

disp('Press q in the open MATLAB figure window to stop ROS2 from the MATLAB side.');
disp('If you close the window by accident, run clear all from the matlab command prompt.')

% so that the window can stop ros nodes
global stopFlag
stopFlag = false;

fig = figure('Name', 'Press "q" to quit');
set(fig, 'KeyPressFcn', @keyPressCallback);

while ~stopFlag
    % Allow figure window and callbacks to process
    drawnow;
end

close all; % close the window so we know that we're done


disp('Shutting down MATLAB ROS nodes...');
clear bendingSub
clear simTimer
clear matlabPubNode
clear matlabSubNode


function keyPressCallback(~, event)
    global stopFlag
    if strcmp(event.Key, 'q')
        stopFlag = true;
    end
end
