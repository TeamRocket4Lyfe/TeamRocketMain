
% Filename: postPrcoessing.m
% File Creation date: 23 September 2017

% Description: This script will process the sensor data received from 
% Team Rocket's P-Sat.
% Contributors: Victoria Skeggs, Louis Habberfield-Short
% Last updated: 29 September 2017

clc; clear; close all;

% Set path to data file
filepath = 'DATA00.CSV';

% Read data from csv file, ignoring headers
data = csvread(filepath,1,0);

% Calculate altitude offset from BMP
altitudeOffset = 4375;

% Read in measurement data
time = data(:, 1)/1000; % convert time to milliseconds
temp = data(:, 2);
pressure = data(:, 3);
altitudeBMP = data(:, 4) + altitudeOffset;
accelX = data(:, 5);
accelY = data(:, 6);
accelZ = data(:, 7);
magX = data(:, 8);
magY = data(:, 9);
magZ = data(:, 10);
gyroX = data(:, 11);
gyroY = data(:, 12);
gyroZ = data(:, 13);
pitch = data(:, 14);
roll = data(:, 15);
heading = data(:, 16);
latitude = data(:, 17);
longitude = data(:, 18);
altitudeGPS = data(:, 19);
speed = data(:, 20);
voltage = data(:, 21);

% Plot each set of measurements against time
measurements = {temp, pressure, altitudeBMP, accelX, accelY, accelZ, magX, magY, ...
    magZ, gyroX, gyroY, gyroZ, pitch, roll, heading, altitudeGPS, speed, voltage};

yLabels = {'temperature (*C)', 'pressure (Pa)', 'altitude from BMP (m)', 'accelX (m/s^2)', ...
    'accelY(m/s^2)', 'accelZ (m/s^2)', 'magX (uT)', 'magY (uT)', ...
    'magZ (uT)', 'gyroX (rad/s)', 'gyroY (rad/s)', 'gyroZ (rad/s)', ...
    'pitch', 'roll', 'heading', 'altitude from GPS (m)', 'speed (m/s)', 'voltage (V)'};

titles = {'Temperature vs Time', 'Pressure vs Time', 'BMP Altitude vs Time', 'AccelX vs Time'...
    , 'AccelY vs Time', 'AccelZ vs Time', 'MagX vs Time'...
    , 'MagY vs Time', 'MagZ vs Time', 'GyroX vs Time', ...
    'GyroY vs Time', 'GyroZ vs Time', 'Pitch vs Time', 'Roll vs Time',... 
    'Heading vs Time', 'GPS Altitude vs Time', 'Speed vs Time', 'Battery Voltage vs Time'};

filenames = {'TempvsTime.png', 'PressvsTime.png', 'BMPAltitude.png', 'AccelXvsTime.png' ...
    'AccelYvsTime.png','AccelZvsTime.png', 'MagXvsTime.png' ...
    'MagYvsTime.png','MagZvsTime.png', 'GyroXvsTime.png' ...
    'GyroYvsTime.png','GyroZvsTime.png', 'PitchvsTime.png', ...
    'RollvsTime.png', 'HeadingvsTime.png', 'GPSAltitude.png', 'Speed.png', 'Voltage.png'};

% Create and save plot of each raw measurement
for i = 1:length(measurements)
    figure(i);
    MakeAndSavePlot(time, 'time (s)', measurements{i}, yLabels{i}, titles{i}, filenames{i});
end

% Smooth datasets
for i = 1:length(measurements)
    current = cell2mat(measurements(i));
    figure(length(measurements) + i);
    
    for j = 1:100
        current = VectorSmooth(current);
    end
    
    % Plot smoothed data
    plot(time, current);
end
