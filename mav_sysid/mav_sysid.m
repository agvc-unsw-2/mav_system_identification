%% user set params

%bag_name = '2019-03-23-18-45-51.bag'

%bag_name = 'testBag.bag'; %'2019-03-23-14-04-15.bag';
%bag_name = '2019-03-23-14-04-15.bag';
%bag_name = '2019-03-23-16-01-20.bag';
%bag_name = '2019-03-23-16-13-18.bag';
%bag_name = '2019-03-25-11-16-09.bag';

%bag_name = 'vrep_hex.bag';
% bag_name = 'iris.bag';

%bag_name = 'vrep_hex_500.bag';
%bag_name = 'aero1_new.bag';
%bag_name = 'nuc2.bag';
bag_name = 'vrep_quad_new_loads_800_new_pq_90.bag';
sys_id_start_time_s = 20.0;
sys_id_end_time_s = 60.0;
delay = 0.0;

if(length(strfind(bag_name,'vrep')) > 0)
    % SIMULATION
    if(length(strfind(bag_name,'quad')) > 0)
        imu_topic = '/vrep_quad1/ground_truth/mavros/imu/data';
        control_topic = '/vrep_quad1/command/roll_pitch_yawrate_thrust';
    elseif(length(strfind(bag_name,'hex')) > 0)
        imu_topic = '/vrep_hex1/ground_truth/mavros/imu/data';
        control_topic = '/vrep_hex1/command/roll_pitch_yawrate_thrust';
    end
else
    % ACTUAL DRONE
    imu_topic = '/mavros/imu/data';
    control_topic = '/mavros/setpoint_raw/roll_pitch_yawrate_thrust';
end

% parameters that must be provided 
% system_mass_kg = 1.95;
% system_mass_kg = 1.65; % vrep_hex

%% Parameters
system_mass_kg = 1.0;

linear_drag_coefficients = [0.01, 0.01, 0]; %default values work for most systems
yaw_gain = 1.0; %default value works for most systems
yaw_damping = 0.95; %default value works for most systems
yaw_omega = 5.0; %default value works for most systems

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

t_start = imu_data.t(1);% + .6;
% t_start = 0
imu_data.t = imu_data.t - t_start;

% 0.05 for aero, 0.10 for f550
attitude_cmd.t = attitude_cmd.t - t_start + delay;

%attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);

[~, duplicates_i] = unique(attitude_cmd.t);
attitude_cmd.t = attitude_cmd.t(duplicates_i);
attitude_cmd.i = attitude_cmd.i(duplicates_i);
attitude_cmd.roll = attitude_cmd.roll(duplicates_i);
attitude_cmd.pitch = attitude_cmd.pitch(duplicates_i);
attitude_cmd.yaw_rate = attitude_cmd.yaw_rate(duplicates_i);
attitude_cmd.thrust = attitude_cmd.thrust(:,duplicates_i);
attitude_cmd.rpy = attitude_cmd.rpy(:,duplicates_i);
%attitude_cmd.thrust = attitude_cmd.thrust(duplicates_i);
%attitude.t(duplicates_i) = [];
%attitude.i(duplicates_i) = [];
%attitude.roll(duplicates_i) = [];
%attitude.pitch(duplicates_i) = [];
%attitude.yaw_rate(duplicates_i) = [];
%attitude.thrust(duplicates_i) = [];
%attitude.rpy(duplicates_i) = [];
%attitude.rpy_interp(duplicates_i) = [];

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
plot(imu_data.t, vecnorm(imu_data.a), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.thrust, '--', 'linewidth', 2);
xlabel('time');
ylabel('???');
title('Thrust');
legend('Acceleration', 'Commanded thrust');
grid on;
ax.FontSize = 16;

%% interpolate to match timestamps
%imu_data.t = imu_data.t + 0.6;
attitude_cmd.rpy_interp = zeros(size(imu_data.rpy));
attitude_cmd.rpy_interp(1,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(1,:), imu_data.t);
attitude_cmd.rpy_interp(2,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(2,:), imu_data.t);
attitude_cmd.rpy_interp(3,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(3,:), imu_data.t);

attitude_cmd.thrust_interp = zeros(size(imu_data.rpy));
attitude_cmd.thrust_interp = interp1(attitude_cmd.t, attitude_cmd.thrust(3,:), imu_data.t);

attitude_cmd.t = imu_data.t;

%% get specified time period
imu_data_old = imu_data.t;
imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
%imu_data.t = imu_data.t - sys_id_start_time_s;
imu_data.rpy = imu_data.rpy(:, imu_data_old > sys_id_start_time_s & imu_data_old < sys_id_end_time_s);
imu_data.a = imu_data.a(:, imu_data_old > sys_id_start_time_s & imu_data_old < sys_id_end_time_s);

attitude_cmd.t = attitude_cmd.t(imu_data_old > sys_id_start_time_s & imu_data_old < sys_id_end_time_s);
%attitude_cmd.t = imu_data.t - sys_id_start_time_s;
attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, imu_data_old > sys_id_start_time_s & imu_data_old < sys_id_end_time_s);
attitude_cmd.thrust_interp = attitude_cmd.thrust_interp(:, imu_data_old > sys_id_start_time_s & imu_data_old < sys_id_end_time_s);

%% get roll and pitch data
dt = mean(diff(imu_data.t));
idx = find(isnan(attitude_cmd.rpy_interp(1,:)));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);

%roll_data = iddata(imu_data.rpy(1,idx(end)+1:end)', attitude_cmd.rpy_interp(1,idx(end)+1:end)', dt);
%pitch_data = iddata(imu_data.rpy(2,idx(end)+1:end)', attitude_cmd.rpy_interp(2,idx(end)+1:end)', dt);

%% Get 1st order models
roll_tf = tfest( roll_data, 1, 0);
pitch_tf = tfest( pitch_data, 1, 0);

disp('=====================================================================');
disp('1st order dynamics'); 
fprintf('roll fit percentage: %f%%\n\n', roll_tf.Report.Fit.FitPercent);
fprintf('pitch fit percentage: %f%%\n', pitch_tf.Report.Fit.FitPercent);
disp('---------------------------------------------------------------------');
disp('Copy the following into your nonlinear_mpc.yaml file')
disp('---------------------------------------------------------------------');
disp('# Controller Parameters:');
fprintf('mass: %f\n',system_mass_kg);
fprintf('roll_time_constant: %f\n', -1/pole(roll_tf));
fprintf('roll_gain: %f\n', dcgain(roll_tf));
fprintf('pitch_time_constant: %f\n', -1/pole(pitch_tf));
fprintf('pitch_gain: %f\n', dcgain(pitch_tf));
fprintf('linear_drag_coefficients: [%f, %f, %f]\n', linear_drag_coefficients(1),linear_drag_coefficients(2),linear_drag_coefficients(3));
fprintf('yaw_gain: %f\n', yaw_gain);
disp('---------------------------------------------------------------------');

roll_tf_first_order = roll_tf;
pitch_tf_first_order = pitch_tf;

%2nd order
roll_tf = tfest( roll_data, 2, 0);
pitch_tf = tfest( pitch_data, 2, 0);

[roll_Wn, roll_damping] = damp(roll_tf);
[pitch_Wn, pitch_damping] = damp(pitch_tf);

disp('=====================================================================');
disp('2nd order dynamics'); 
fprintf('roll fit percentage: %f%%\n\n', roll_tf.Report.Fit.FitPercent);
fprintf('pitch fit percentage: %f%%\n', pitch_tf.Report.Fit.FitPercent);
disp('---------------------------------------------------------------------');
disp('Copy the following into your disturbance_observer.yaml file');
disp('---------------------------------------------------------------------');
disp('  #model from system identification (2nd order attitude model)');
fprintf('  drag_coefficients: [%f, %f, %f]\n', linear_drag_coefficients(1),linear_drag_coefficients(2),linear_drag_coefficients(3));
fprintf('  roll_gain: %f\n', dcgain(roll_tf));
fprintf('  roll_damping: %f\n', roll_damping(1));
fprintf('  roll_omega: %f\n', roll_Wn(1));
fprintf('  pitch_gain: %f\n', dcgain(pitch_tf));
fprintf('  pitch_damping: %f\n', pitch_damping(1));
fprintf('  pitch_omega: %f\n', pitch_Wn(1));
fprintf('  yaw_gain: %f\n', yaw_gain);
fprintf('  yaw_damping: %f\n', yaw_damping);
fprintf('  yaw_omega: %f\n', yaw_omega);
disp('---------------------------------------------------------------------');

%thrust
acc_thrust_ratio = (vecnorm(imu_data.a)-9.81) ./ attitude_cmd.thrust_interp;
acc_thrust_ratio = median(acc_thrust_ratio(acc_thrust_ratio > 0.0)) / system_mass_kg;

disp('=====================================================================');
disp('pixhawk thrust settings'); 
disp('---------------------------------------------------------------------');
disp('Copy the following into your px4_config.yaml file')
disp('---------------------------------------------------------------------');
disp('# setpoint_raw');
disp('setpoint_raw:');
fprintf('  thrust_scaling_factor: %f\n',acc_thrust_ratio);
fprintf('  system_mass_kg: %f\n', system_mass_kg);
fprintf('  yaw_rate_scaling_factor: %f\n', yaw_gain);
disp('---------------------------------------------------------------------');

roll_tf_second_order = roll_tf;
pitch_tf_second_order = pitch_tf;

%% plot input and actual
figure();
ax = axes;
plot(roll_data);
xlabel('time (s)');
ylabel('roll (rad)');
title('roll angle');
grid on;
ax.FontSize = 16;

figure();
ax = axes;
plot(pitch_data);
xlabel('time (s)');
ylabel('pitch (rad)');
title('pitch angle');
grid on;
ax.FontSize = 16;

%% Setup simulated output time
time_to_plot_start = 5;
time_to_plot_end = 30;
start_elem = floor(time_to_plot_start / dt) + 1;
end_elem = floor(time_to_plot_end / dt) + 1;
t_sim = 0:dt:ceil(length(roll_data.u)*dt);
t_sim = t_sim(1:length(roll_data.u));

%% Plot simulated roll (1st order)
roll_sim = sim(roll_tf_first_order, roll_data.u);
figure(); hold on;
plot(t_sim(start_elem:end_elem), roll_data.y(start_elem:end_elem), 'linewidth', 2);
plot(t_sim(start_elem:end_elem), roll_sim(start_elem:end_elem), '--r', 'linewidth', 2);
legend('roll', 'simulated roll');
xlabel('time (s)');
ylabel('roll (rad)');
title('Modelled roll and actual roll (1st order model)')
xlim([time_to_plot_start, time_to_plot_end]);
ylim([min(roll_data.y(start_elem:end_elem)) - 0.25, max(roll_data.y(start_elem:end_elem)) + 0.25]);

%% Plot simulated pitch (1st order)
pitch_sim = sim(pitch_tf_first_order, pitch_data.u);
figure(); hold on;
plot(t_sim(start_elem:end_elem), pitch_data.y(start_elem:end_elem), 'linewidth', 2);
plot(t_sim(start_elem:end_elem), pitch_sim(start_elem:end_elem), '--r', 'linewidth', 2);
legend('pitch', 'simulated roll');
xlabel('time (s)');
ylabel('roll (rad)');
title('Modelled pitch and actual pitch (1st order model)')
xlim([time_to_plot_start, time_to_plot_end]);
ylim([min(pitch_data.y(start_elem:end_elem)) - 0.25, max(pitch_data.y(start_elem:end_elem)) + 0.25]);

%% Plot simulated roll (2nd order)
roll_sim = sim(roll_tf_second_order, roll_data.u);
figure(); hold on;
plot(t_sim(start_elem:end_elem), roll_data.y(start_elem:end_elem), 'linewidth', 2);
plot(t_sim(start_elem:end_elem), roll_sim(start_elem:end_elem), '--r', 'linewidth', 2);
legend('roll', 'simulated roll');
xlabel('time (s)');
ylabel('roll (rad)');
title('Modelled roll and actual roll (2nd order model)')
xlim([time_to_plot_start, time_to_plot_end]);
ylim([min(roll_data.y(start_elem:end_elem)) - 0.25, max(roll_data.y(start_elem:end_elem)) + 0.25]);

%% Plot simulated pitch (2nd order)
pitch_sim = sim(pitch_tf_second_order, pitch_data.u);
figure(); hold on;
plot(t_sim(start_elem:end_elem), pitch_data.y(start_elem:end_elem), 'linewidth', 2);
plot(t_sim(start_elem:end_elem), pitch_sim(start_elem:end_elem), '--r', 'linewidth', 2);
legend('pitch', 'simulated roll');
xlabel('time (s)');
ylabel('roll (rad)');
title('Modelled pitch and actual pitch (2nd order model)')
xlim([time_to_plot_start, time_to_plot_end]);
ylim([min(pitch_data.y(start_elem:end_elem)) - 0.25, max(pitch_data.y(start_elem:end_elem)) + 0.25]);