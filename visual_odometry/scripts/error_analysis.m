clear; clc; close all;

path = "C:\Users\david\Downloads\result_vio_2.bag";
bag = rosbag(path);

VOPoseBag = select(bag, 'Topic', '/pose_topic');
truthPoseBag = select(bag, 'Topic', '/dji/vicon/dji_m100/dji_m100');

VOAttitudeBag = select(bag, 'Topic', '/vo_att');
truthAttitudeBag = select(bag, 'Topic', '/truth_att');

VOPoseMsgs = readMessages(VOPoseBag);
truthPoseMsgs = readMessages(truthPoseBag);

VOAttitudeMsgs = readMessages(VOAttitudeBag);
truthAttitudeMsgs = readMessages(truthAttitudeBag);

ts1 = timeseries(VOPoseBag);
ts2 = timeseries(truthPoseBag);
ts3 = timeseries(VOAttitudeBag);
ts4 = timeseries(truthAttitudeBag);


t1 = ts1.Time; % VO pose time
t2 = ts2.Time; % Truth pose time

t3 = ts3.Time; % VO attitude time
t4 = ts4.Time; % Truth attitude time



% Offset the time values to start from 0
t1_ = t1 - t2(1);
t2_ = t2 - t2(1);
t3_ = t3 - t2(1);
t4_ = t4 - t2(1);

% Match timestamps between VO pose and truth pose
k = 1;
if size(t2, 1) > size(t1, 1)
    for i = 1:size(t1, 1)
        [val, indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx_VO2truth(k, :) = [i, indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2, 1)
        [val, indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx_VO2truth(k, :) = [indx, i];
            k = k + 1;
        end
    end
end


% Match timestamps between VO attitude and truth attitude
k = 1;
if size(t4, 1) > size(t3, 1)
    for i = 1:size(t3, 1)
        [val, indx] = min(abs(t3(i) - t4));
        if val <= 0.1
            idx_VO2truthAtt(k, :) = [i, indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t4, 1)
        [val, indx] = min(abs(t4(i) - t3));
        if val <= 0.1
            idx_VO2truthAtt(k, :) = [indx, i];
            k = k + 1;
        end
    end
end


VOPoseError = zeros(size(idx_VO2truth, 1), 3);
VOAttitudeError = zeros(size(idx_VO2truthAtt, 1), 3);

%%
% Initialize arrays to store pose data
VOPose = zeros(size(VOPoseMsgs, 1), 3);

VOAttitude = zeros(size(VOAttitudeMsgs, 1), 3);

truthAttitude = zeros(size(truthAttitudeMsgs, 1), 3);

truthPose = zeros(size(truthPoseMsgs, 1), 3);

% Read all messages and store relevant pose and attitude data
for i = 1:length(VOPoseMsgs)
    VOPose(i, :) = [ ...
        VOPoseMsgs{i}.Pose.Position.X, ...
        VOPoseMsgs{i}.Pose.Position.Y, ...
        VOPoseMsgs{i}.Pose.Position.Z];
end

for i = 1:length(truthPoseMsgs)
    truthPose(i, :) = [ ...
        truthPoseMsgs{i}.Transform.Translation.X, ...
        truthPoseMsgs{i}.Transform.Translation.Y, ...
        truthPoseMsgs{i}.Transform.Translation.Z];
end


for i = 1:length(VOAttitudeMsgs)
    VOAttitude(i, :) = [ ...
        VOAttitudeMsgs{i}.Vector.X, ...
        VOAttitudeMsgs{i}.Vector.Y, ...
        VOAttitudeMsgs{i}.Vector.Z];
end


for i = 1:length(truthAttitudeMsgs)
    truthAttitude(i, :) = [ ...
        truthAttitudeMsgs{i}.Vector.X, ...
        truthAttitudeMsgs{i}.Vector.Y, ...
        truthAttitudeMsgs{i}.Vector.Z];
end

%%

% Calculate VO pose error
for i = 1:size(idx_VO2truth, 1)
    VOPoseError(i, 1) = abs(VOPoseMsgs{idx_VO2truth(i, 1)}.Pose.Position.X - ...
        truthPoseMsgs{idx_VO2truth(i, 2)}.Transform.Translation.X);
    VOPoseError(i, 2) = abs(VOPoseMsgs{idx_VO2truth(i, 1)}.Pose.Position.Y - ...
        truthPoseMsgs{idx_VO2truth(i, 2)}.Transform.Translation.Y);
    VOPoseError(i, 3) = abs(VOPoseMsgs{idx_VO2truth(i, 1)}.Pose.Position.Z - ...
        truthPoseMsgs{idx_VO2truth(i, 2)}.Transform.Translation.Z);
end


% Calculate VO attitude error
for i = 1:size(idx_VO2truthAtt, 1)
    VOAttitudeError(i, 1) = abs(VOAttitudeMsgs{idx_VO2truthAtt(i, 1)}.Vector.X - ...
        truthAttitudeMsgs{idx_VO2truthAtt(i, 2)}.Vector.X);
    VOAttitudeError(i, 2) = abs(VOAttitudeMsgs{idx_VO2truthAtt(i, 1)}.Vector.Y - ...
        truthAttitudeMsgs{idx_VO2truthAtt(i, 2)}.Vector.Y);
    VOAttitudeError(i, 3) = abs(VOAttitudeMsgs{idx_VO2truthAtt(i, 1)}.Vector.Z - ...
        truthAttitudeMsgs{idx_VO2truthAtt(i, 2)}.Vector.Z);
end


%% Plots
% Define the size of the plot points
pointSize = 3;
fontSize1 = 19;
fontSize2 = 19;
pointSize2 = 10;

% VO X - Error
figure;
subplot(4, 1, 1)
plot(t1_, VOPoseError(:, 1), 'LineWidth', pointSize);
%title("VO X - Error");
xlabel("Time [s]");
ylabel("X Error [m]");
grid;
rmsVOPoseErrorX = rms(VOPoseError(:, 1));
fprintf("RMS VO Position - X Error = %3.5fm\n", rmsVOPoseErrorX);
fprintf("Max VO Position - X Error = %3.5fm\n", max(VOPoseError(:, 1)));

% VO Y - Error
subplot(4, 1, 2)
plot(t1_, VOPoseError(:, 2), 'LineWidth', pointSize);
%title("VO Y - Error");
xlabel("Time [s]");
ylabel("Y Error [m]");
grid;
rmsVOPoseErrorY = rms(VOPoseError(:, 2));
fprintf("RMS VO Position - Y Error = %3.5fm\n", rmsVOPoseErrorY);
fprintf("Max VO Position - Y Error = %3.5fm\n", max(VOPoseError(:, 2)));

% VO Z - Error
subplot(4, 1, 3)
plot(t1_, VOPoseError(:, 3), 'LineWidth', pointSize);
%title("VO Z - Error");
xlabel("Time [s]");
ylabel("Z Error [m]");
grid;
rmsVOPoseErrorZ = rms(VOPoseError(:, 3));
fprintf("RMS VO Position - Z Error = %3.5fm\n", rmsVOPoseErrorZ);
fprintf("Max VO Position - Z Error = %3.5fm\n", max(VOPoseError(:, 3)));

% VO Pose Error
subplot(4, 1, 4)
plot(t1_, sqrt(VOPoseError(:, 1).^2 + VOPoseError(:, 2).^2 + VOPoseError(:, 3).^2), ...
    'LineWidth', pointSize);
%title("VO Position Error");
xlabel("Time [s]");
ylabel("3D Position Error [m]");
grid;
rmsVOPoseError = rms(sqrt(VOPoseError(:, 1).^2 + VOPoseError(:, 2).^2 + VOPoseError(:, 3).^2));
fprintf("RMS VO Position Error = %3.5fm\n", rmsVOPoseError);
fprintf("Max VO Position Error = %3.5fm\n", max(sqrt(VOPoseError(:, 1).^2 + VOPoseError(:, 2).^2 + VOPoseError(:, 3).^2)));


% VO Roll - Error
figure;
subplot(3, 1, 1);
plot(t3_, VOAttitudeError(:, 1), 'LineWidth', pointSize);
%title("VO Roll - Error");
xlabel("Time [s]");
ylabel("Roll Error [rad]");
grid;
rmsVOAttitudeErrorRoll = rms(VOAttitudeError(:, 1));
fprintf("RMS VO Roll Error = %3.5frad\n", rmsVOAttitudeErrorRoll);
fprintf("Max VO Roll Error = %3.5frad\n", max(VOAttitudeError(:, 1)));

% VO Pitch - Error
subplot(3, 1, 2);
plot(t3_, VOAttitudeError(:, 2), 'LineWidth', pointSize);
%title("VO Pitch - Error");
xlabel("Time [s]");
ylabel("Pitch Error [rad]");
grid;
rmsVOAttitudeErrorPitch = rms(VOAttitudeError(:, 2));
fprintf("RMS VO Pitch Error = %3.5frad\n", rmsVOAttitudeErrorPitch);
fprintf("Max VO Pitch Error = %3.5frad\n", max(VOAttitudeError(:, 2)));

% VO Yaw - Error
subplot(3, 1, 3);
plot(t3_, VOAttitudeError(:, 3), 'LineWidth', pointSize);
%title("VO Yaw - Error");
xlabel("Time [s]");
ylabel("Yaw Error [rad]");
grid;
rmsVOAttitudeErrorYaw = rms(VOAttitudeError(:, 3));
fprintf("RMS VO Yaw Error = %3.5frad\n", rmsVOAttitudeErrorYaw);
fprintf("Max VO Yaw Error = %3.5frad\n", max(VOAttitudeError(:, 3)));


%%
% Plot VO Position and Truth Position vs Time
figure;
subplot(3, 1, 1);
plot(t1_, VOPose(:, 1), 'r-', 'LineWidth', pointSize);
hold on;
plot(t2_, truthPose(:, 1), 'b:', 'LineWidth', pointSize);
xlabel("Time [s]");
ylabel("X [m]");
legend("VO Position", "Truth Position", "FontSize", fontSize1);
ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;

subplot(3, 1, 2);
plot(t1_, VOPose(:, 2), 'r-', 'LineWidth', pointSize);
hold on;

plot(t2_, truthPose(:, 2), 'b:', 'LineWidth', pointSize);
%%title("VO Y and Truth Y vs Time");
xlabel("Time [s]");
ylabel("Y [m]");
ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;

subplot(3, 1, 3);
plot(t1_, VOPose(:, 3), 'r-', 'LineWidth', pointSize);
hold on;

plot(t2_, truthPose(:, 3), 'b:', 'LineWidth', pointSize);
xlabel("Time [s]");
ylabel("Z [m]");
ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;

% Plot VO Attitude and Truth Attitude vs Time
figure;
subplot(3, 1, 1);
plot(t3_, VOAttitude(:, 1), 'r-', 'LineWidth', pointSize);
hold on;

plot(t4_, truthAttitude(:, 1), 'b:', 'LineWidth', pointSize);
%title("VO and Truth Attitude vs Time");
xlabel("Time [s]");
ylabel("Roll [rad]");
xlim([0, 50]);
legend("VO Attitude", "Truth Attitude", "FontSize", fontSize1);
ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;

subplot(3, 1, 2);
plot(t3_, VOAttitude(:, 2), 'r-', 'LineWidth', pointSize);
hold on;

plot(t4_, truthAttitude(:, 2), 'b:', 'LineWidth', pointSize);

xlabel("Time [s]");
ylabel("Pitch [rad]");
xlim([0, 50]);

ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;

subplot(3, 1, 3);
plot(t3_, VOAttitude(:, 3), 'r-', 'LineWidth', pointSize);
hold on;

plot(t4_, truthAttitude(:, 3), 'b:', 'LineWidth', pointSize);

xlabel("Time [s]");
ylabel("Yaw [rad]");
xlim([0, 50]);

ax = gca; 
ax.FontSize = fontSize2;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontWeight = 'bold';
grid;