%% user set params

bag_name = '2019-01-27-23-29-35.bag';
imu_topic = '/simulation/uav1/ground_truth/mavros/imu/data';
control_topic = '/vrep_hex/command/roll_pitch_yawrate_thrust';

sys_id_start_time_s = 10;
sys_id_end_time_s = 150;

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

close all;
clc;

bag = ros.Bag(bag_name);
bag.info

imu_data = readImu(bag, imu_topic);
attitude_cmd = readCommandRollPitchYawRateThrust(bag, control_topic);
%%

imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);

t_start = imu_data.t(1);
imu_data.t = imu_data.t - t_start;
attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
%% plot
figure(1);
ax = axes;
plot(imu_data.t, imu_data.rpy(1,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(1,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('roll [rad]');
title('roll angle');
legend('\phi imu', '\phi cmd');
grid on;
ax.FontSize = 16;

figure(2);
ax = axes;
plot(imu_data.t, imu_data.rpy(2,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(2,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('\theta [rad]');
title('pitch angle');
legend('\theta imu', '\theta cmd');
grid on;
ax.FontSize = 16;

figure(3);
ax = axes;
plot(imu_data.t, imu_data.a(3, :), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.thrust, '--', 'linewidth', 2);
xlabel('time');
ylabel('???');
title('Thrust');
legend('Acceleration', 'Commanded thrust');
grid on;
ax.FontSize = 16;

%% sysid

% Get uqique
[att_cmd_rpy_unique_1, i_1] = unique(attitude_cmd.rpy(1,:));
att_t_1 = attitude_cmd.t(i_1);
[att_cmd_rpy_unique_2, i_2] = unique(attitude_cmd.rpy(2,:));
att_t_2 = attitude_cmd.t(i_2);
[att_cmd_rpy_unique_3, i_3] = unique(attitude_cmd.rpy(3,:));
att_t_3 = attitude_cmd.t(i_3);
imu_t = imu_data.t;

% Start sysid
attitude_cmd_rpy_unique_interp = zeros(size(imu_data.rpy));
attitude_cmd_rpy_unique_interp(1,:) = interp1(att_t_1, att_cmd_rpy_unique_1, imu_data.t);
attitude_cmd_rpy_unique_interp(2,:) = interp1(att_t_2, att_cmd_rpy_unique_2, imu_data.t);
attitude_cmd_rpy_unique_interp(3,:) = interp1(att_t_3, att_cmd_rpy_unique_3, imu_data.t);

att_t_1 = imu_data.t;
att_t_2 = imu_data.t;
att_t_3 = imu_data.t;

imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
imu_data.rpy = imu_data.rpy(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);

att_t_1 = att_t_1(att_t_1 > sys_id_start_time_s & att_t_1 < sys_id_end_time_s);
attitude_cmd_rpy_unique_interp(1, :) = attitude_cmd_rpy_unique_interp(1, att_t_1 > sys_id_start_time_s & att_t_1 < sys_id_end_time_s);
att_t_2 = att_t_2(att_t_2 > sys_id_start_time_s & att_t_2 < sys_id_end_time_s);
attitude_cmd_rpy_unique_interp(2, :) = attitude_cmd_rpy_unique_interp(2, att_t_2 > sys_id_start_time_s & att_t_2 < sys_id_end_time_s);
att_t_3 = att_t_3(att_t_3 > sys_id_start_time_s & att_t_3 < sys_id_end_time_s);
attitude_cmd_rpy_unique_interp(3, :) = attitude_cmd_rpy_unique_interp(3, att_t_3 > sys_id_start_time_s & att_t_3 < sys_id_end_time_s);

dt = mean(diff(imu_data.t));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd_rpy_unique_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd_rpy_unique_interp(2,:)', dt);

roll_tf = tfest( roll_data, 1, 0);
pitch_tf = tfest( pitch_data, 1, 0);
disp('======================');
disp('roll 1st order dynamics'); 
% roll_tf
fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
fprintf('roll_time_constant: %f\n\n', -1/pole(roll_tf));
fprintf('fit percentage: %f %%\n', roll_tf.Report.Fit.FitPercent);
disp('----------------------');
disp('pitch 1st order dynamics'); 
% pitch_tf
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
fprintf('pitch_time_constant: %f\n\n', -1/pole(pitch_tf));
fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);

%2nd order
roll_tf = tfest( roll_data, 2, 0);
pitch_tf = tfest( pitch_data, 2, 0);
disp('----------------------');
disp('roll 2st order dynamics:'); 
% roll_tf
[Wn, damping] = damp(roll_tf);
fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
fprintf('roll_damping_coef: %f\n\n', damping(1));
fprintf('roll_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', roll_tf.Report.Fit.FitPercent);

disp('----------------------');
disp('pitch 2st order dynamics:'); 
% pitch_tf
[Wn, damping] = damp(pitch_tf);
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
fprintf('pitch_damping_coef: %f\n\n', damping(1));
fprintf('pitch_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);



%% 
