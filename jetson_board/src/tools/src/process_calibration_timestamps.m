%% Script to process imu_timetamp and mcu_timestamp data from IMU_CAL phase

% Author: Kevin Gabriel Alvarez
% Date: August 25th, 2026
% Email: kegabriel@ethz.ch

clear;
clc;

%% User defined variables

csv_file = "../../../csv/25_8_2026_10_min.csv";

%% Load data
raw_data = readmatrix(csv_file);

imu_timestamps = raw_data(:,1)*23.5849057;
mcu_timestamps = raw_data(:,2);

%% Plot results

figure (1);
clf;
plot(imu_timestamps, mcu_timestamps, 'k.', 'LineWidth', 2);

xlabel('IMU timestamps [us]');
ylabel('MCU timestamps [us]');
grid on;

error = (imu_timestamps - mcu_timestamps)*1/10^6; % [s]
N = length(error);

figure (2);
clf;
plot(linspace(0, N-1, N), error, '-k', 'LineWidth', 2);
xlabel('Sample number');
ylabel('Error [s]');
title(['Max error = ', num2str(max(error)), ...
       ' s, min error = ', num2str(min(error)), ...
       ' s, total deviation = ', num2str(abs(max(error) - min(error))), ' s']);
grid on;

figure (3);
clf;
plot(linspace(0, N-2, N-1), diff(mcu_timestamps), '-r', 'LineWidth', 2);
hold on;
plot(linspace(0, N-2, N-1), diff(imu_timestamps), '--b', 'LineWidth', 2);
xlabel('Sample number');
ylabel('Period between subsequent samples [s]');
legend('MCU timestamps', 'IMU timestamps');
grid on;