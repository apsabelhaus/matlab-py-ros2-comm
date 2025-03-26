% Initialize ROS core
rosinit;

% --- Setup: Publisher to /control_values ---
controlPub = rospublisher('/control_values', 'std_msgs/Float64MultiArray');
controlMsg = rosmessage(controlPub);

% --- Setup: Subscriber to /bending_angles ---
bendingSub = rossubscriber('/bending_angles', 'std_msgs/Float64MultiArray');

% --- Rates ---
pubRate = 1;    % 1 Hz publish
loopRate = robotics.Rate(5);   % 5 Hz subscriber

% --- Initialize timing ---
lastPubTime = rostime('now');
lastSubTime = rostime('now');
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
    currentTime = rostime('now');
    if currentTime.Sec - lastPubTime.Sec >= 1.0/pubRate
        controlMsg.Data = rand(2,1);
        send(controlPub, controlMsg);
        disp(['[Publish] control_values: ', mat2str(controlMsg.Data')]);
        lastPubTime = currentTime;
    end
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