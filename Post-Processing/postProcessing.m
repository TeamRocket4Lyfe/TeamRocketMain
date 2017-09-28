% Filename: postPrcoessing.m
% Original author: Victoria Pickett
% File Creation date: 23 September 2017

% Description: This script will process the sensor data received from 
% Team Rocket's P-Sat.
% Contributors: Victoria Pickett, Louis Habberfield-Short
% Last updated: 29 September 2017

clc; clear; close all;

% Set path to data file
filepath = 'DATA00.csv';

% Read data from csv file, ignoring headers
data = csvread(filepath,1,0);

% Allocate measurement data
time = data(:, 1)/1000;
temp = data(:, 2);
pressure = data(:, 3);
accelX = data(:, 4);
accelY = data(:, 5);
accelZ = data(:, 6);
magX = data(:, 7);
magY = data(:, 8);
magZ = data(:, 9);
gyroX = data(:, 10);
gyroY = data(:, 11);
gyroZ = data(:, 12);
pitch = data(:, 13);
roll = data(:, 14);
heading = data(:, 15);

% Plot each set of measurements against time
measurements = {temp, pressure, accelX, accelY, accelZ, magX, magY, ...
    magZ, gyroX, gyroY, gyroZ, pitch, roll, heading};
yLabels = {'temperature (*C)', 'pressure (Pa)', 'accelX (m/s^2)', ...
    'accelY(m/s^2)', 'accelZ (m/s^2)', 'magX (uT)', 'magY (uT)', ...
    'magZ (uT)', 'gyroX (rad/s)', 'gyroY (rad/s)', 'gyroZ (rad/s)', ...
    'pitch', 'roll', 'heading'};

titles = {'Temperature vs Time', 'Pressure vs Time', 'AccelX vs Time'...
    , 'AccelY vs Time', 'AccelZ vs Time', 'MagX vs Time'...
    , 'MagY vs Time', 'MagZ vs Time', 'GyroX vs Time', ...
    'GyroY vs Time', 'GyroZ vs Time', 'Pitch vs Time', 'Roll vs Time',... 
    'Heading vs Time'};

filenames = {'TempvsTime.png', 'PressvsTime.png', 'AccelXvsTime.png' ...
    'AccelYvsTime.png','AccelZvsTime.png', 'MagXvsTime.png' ...
    'MagYvsTime.png','MagZvsTime.png', 'GyroXvsTime.png' ...
    'GyroYvsTime.png','GyroZvsTime.png', 'PitchvsTime.png', ...
    'RollvsTime.png', 'HeadingvsTime.png'};

for i = 1:length(measurements)
    figure(i);
    MakeAndSavePlot(time, 'time (s)', measurements{i}, yLabels{i}, titles{i}, filenames{i});
end