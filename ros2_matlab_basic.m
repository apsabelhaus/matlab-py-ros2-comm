% Modified 2025-03-26 for ROS2. -Drew

%%% Setup just to make sure we start at the same state each time.
clear all
close all
clc;

disp('ROS2 Matlab demo for communication with Python.');
disp('(C) Soft Robotics Control Lab at Boston University, 2025');
disp('creating the nodes / starting ros2...');

%%% Initialize ROS2 nodes for the MATLAB side
% we are this node for sending control commands
matlabPubNodeName = "matlab_cmder";
matlabPubNode = ros2node(matlabPubNodeName);
% we are this node for receiving joint angles
matlabSubNodeName = "matlab_receiver";
matlabSubNode = ros2node(matlabSubNodeName);

% --- Setup: Publisher to /control_values ---

pubWait = 2;

disp("Starting the publisher, waiting " + string(pubWait) + " seconds...");

controlPub = ros2publisher(matlabPubNode, '/control_values', 'std_msgs/Float64MultiArray');
controlMsg = ros2message(controlPub);
pause(pubWait); %wait for some time to register publisher on the network

% --- Setup: Subscriber to /bending_angles ---

subStarted = 0;
substartwait = 5;

% currently unused, but this variable would contain the data received from
% ros
global bendingVec;

disp('Attempting to start the subscriber...')
while ~subStarted
    try
        bendingSub = ros2subscriber(matlabSubNode, '/bending_angles', @ezloophwROS2BendingAngleCallback);
        subStarted = 1;
    catch
        disp("ERROR! You need to start the bending_angles publisher in python.");
        disp("Waiting another " + string(substartwait) + " seconds, trying again...");
        pause(substartwait);
    end
end

% --- Rates ---
pubRate = 1;    % 1 Hz publish

% --- Initialize timing ---

%%%% For the publisher:
% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of pubRate.
timerHandles.controlPub = controlPub;
timerHandles.controlPubmsg = controlMsg;
simTimer = ezloophwROS2Timer(pubRate, {@ezloophwROS2SimTimer,timerHandles});

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
