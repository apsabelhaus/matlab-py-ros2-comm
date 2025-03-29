% Modified 2025-03-26 for ROS2. -Drew

% Initialize ROS core
% rosinit;

%%% Initialize ROS2 nodes for the MATLAB side
% we are this node for sending control commands
matlabPubNodeName = "matlab_cmder";
matlabPubNode = ros2node(matlabPubNodeName);
% we are this node for receiving joint angles
matlabSubNodeName = "matlab_receiver";
matlabSubNode = ros2node(matlabSubNodeName);

% --- Setup: Publisher to /control_values ---
% controlPub = rospublisher('/control_values', 'std_msgs/Float64MultiArray');
% controlMsg = rosmessage(controlPub);

controlPub = ros2publisher(matlabPubNode, '/control_values', 'std_msgs/Float64MultiArray');
controlMsg = ros2message(controlPub);
pause(10); %wait for some time to register publisher on the network

% --- Setup: Subscriber to /bending_angles ---
% bendingSub = rossubscriber('/bending_angles', 'std_msgs/Float64MultiArray');
% bendingSub = ros2subscriber(matlabSubNode, '/bending_angles', 'std_msgs/Float64MultiArray');
% debugging
bendingSub = ros2subscriber(matlabSubNode, '/control_values', 'std_msgs/Float64MultiArray');

% --- Rates ---
pubRate = 1;    % 1 Hz publish
% loopRate = robotics.Rate(5);   % 5 Hz subscriber

% --- Initialize timing ---

%%%% For the publisher:
% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of pubRate.
timerHandles.controlPub = controlPub;
timerHandles.controlPubmsg = controlMsg;
simTimer = ezloophwROS2Timer(pubRate, {@ezloophwROS2SimTimer,timerHandles});

disp('breakpoint');

% lastPubTime = rostime('now');
% lastSubTime = rostime('now');
disp('ROS active. Press "q" to stop.');

fig = figure('Name', 'Press "q" to quit');
set(fig, 'KeyPressFcn', @keyPressCallback);

% Data log: [timestamp, angle1, angle2]
bendingLog = [];
t = 0;
% Global flag
global stopFlag
stopFlag = false;

while ~stopFlag
    % Allow figure window and callbacks to process
    drawnow;
    % --- Publish control_values at 1 Hz ---
    % currentTime = rostime('now');
    % if currentTime.Sec - lastPubTime.Sec >= 1.0/pubRate
    %     controlMsg.Data = rand(2,1);
    %     send(controlPub, controlMsg);
    %     disp(['[Publish] control_values: ', mat2str(controlMsg.Data')]);
    %     lastPubTime = currentTime;
    % end
    % --- Read latest from bending_angles ---
    bendingMsg = bendingSub.LatestMessage;
        if ~isempty(bendingMsg)
            bendingVec = bendingMsg.Data(:)';  % Row vector
            %t = double(bendingMsg.Header.Stamp.Sec) + double(bendingMsg.Header.Stamp.Nsec) * 1e-9;
            if t == 0  % fallback if header timestamp is missing
                t_zero = double(rostime('now').Sec) + double(rostime('now').Nsec) * 1e-9;
                t = 0.001;
            else
                t = double(rostime('now').Sec) + double(rostime('now').Nsec) * 1e-9 - t_zero;
            end
            logRow = [t, bendingVec];  % [timestamp, angle1, angle2]
            bendingLog(end+1, :) = logRow;
            disp(['[Subscribe] bending_angles: ', mat2str(bendingVec)]);
        end
    % Wait to maintain loop timing
    waitfor(loopRate);
end
disp('Shutting down ROS...');
rosshutdown;
close all;
% Save to CSV
writematrix(bendingLog, 'bendingLog.csv');
disp('Logged data saved to bendingLog.csv');

function keyPressCallback(~, event)
    global stopFlag
    if strcmp(event.Key, 'q')
        stopFlag = true;
    end
end