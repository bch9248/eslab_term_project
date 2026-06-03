%% Clean Offline Plotter
clear; clc; close all;

% 1. Configuration
fs = 16000;
fftSize = 512;
binsToSend = 256;
freqAxis = (0:binsToSend-1) * (fs / fftSize); 

% 2. Load the .csv matrix (Dimensions will now be 256 x N)
recordedData = readmatrix('stm32_fft_record_Run1.csv');

% 3. Plot the 2D Graph
figure('Name', 'Captured Sample Profile', 'NumberTitle', 'off');
plot(freqAxis, recordedData, 'LineWidth', 1.2);

% 4. Add Labels and Styling
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Magnitude', 'FontSize', 12, 'FontWeight', 'bold');
title('Recorded Frames Overlay', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, freqAxis(end)]);
grid on;