%% STM32 Real-Time FFT Binary Serial Plotter (Calibrated Live Stream Only)
clear; clc; close all;

%% CONFIGURATION
portName = "COM8";       
baudRate = 2000000;
AUDIO_REC = 1024;
fftSize = AUDIO_REC / 2;                    % AUDIO_REC / 2
numBins = fftSize / 2;            % 256 bins total calculated by MCU
binsToSend = fftSize / 2;                 % Full 256 floats expected per packet
fs = 16000;                       % Sampling Frequency

% Calculate frequency resolution per bin
freqAxis = (0:binsToSend-1) * (fs / fftSize); 

%% SETUP GRAPHICS
fig = figure('Name', 'STM32 Live FFT Spectrum', 'NumberTitle', 'off');
ax = axes('Parent', fig);
grid(ax, 'on');
xlabel(ax, 'Frequency (Hz)');
ylabel(ax, 'Magnitude');
title(ax, 'Real-Time FFT Spectrum (Robust Non-Blocking Stream)');

% Initialize spectrum plot line with zeros
fftCurve = plot(ax, freqAxis, zeros(1, binsToSend), 'Color', [0 0.5 0.8], 'LineWidth', 1.5);
xlim(ax, [0, freqAxis(end)]);
ylim(ax, [0, 10000000]); % Adjust max height based on your mic's sensitivity level

%% INITIALIZE SERIAL PORT
try
    device = serialport(portName, baudRate);
    flush(device);
    fprintf('Connected to %s. Syncing data stream...\n', portName);
catch ME
    error('Could not connect. Ensure PuTTY is completely CLOSED! Info: %s', ME.message);
end

% Clean up hardware handle when script stops or window is closed
cleanupObj = onCleanup(@() delete(device));

%% STREAM ALIGNMENT SETUP
% Define the calibration sync header (must match the 4 bytes sent by STM32)
SYNC_HEADER = [0xAA, 0xBB, 0xCC, 0xDD];
headerSize = length(SYNC_HEADER);
dataSizeInBytes = binsToSend * 4;          % 256 floats * 4 bytes = 1024 bytes
packetSize = dataSizeInBytes + headerSize; % 1028 bytes total per frame

% Persistent local stream buffer to process incoming bytes sequentially
serialBuffer = uint8([]); 

%% MAIN PROCESSING LOOP
while ishandle(fig)
    
    % --- NON-BLOCKING DATA ACQUISITION ---
    bytesAvailable = device.NumBytesAvailable;
    if bytesAvailable > 0
        % Instantly ingest whatever bytes are available in the serial hardware stack
        rawData = read(device, bytesAvailable, "uint8");
        serialBuffer = [serialBuffer, rawData]; 
    end
    
    % Process chunks inside the buffer whenever it has enough bytes for a full packet window
    while length(serialBuffer) >= packetSize
        
        % Locate the header byte sequence pattern anywhere inside our array
        headerIdx = strfind(serialBuffer, SYNC_HEADER);
        
        if isempty(headerIdx)
            % Header sequence not found. Retain only structural tail fragments 
            % to catch occurrences split over streaming windows.
            if length(serialBuffer) >= headerSize
                serialBuffer = serialBuffer(end - headerSize + 2 : end);
            end
            break; % Return to the serial stream line to ingest more bytes
        end
        
        % If the earliest matched header isn't at the very front (index 1),
        % drop the stale leading garbage data preceding it.
        if headerIdx(1) > 1
            serialBuffer = serialBuffer(headerIdx(1):end);
            continue; % Immediately restart evaluation with aligned indexing
        end
        
        % Confirm if the entire trailing float payload has entered our local buffer array
        if length(serialBuffer) < packetSize
            break; % Incomplete payload frame. Defer processing until next data read.
        end
        
        % Extract the exact raw float byte block following our sync header
        rawFrameBytes = serialBuffer(headerSize + 1 : packetSize);
        
        % Pop out the handled data frame package safely from the local queue
        serialBuffer = serialBuffer(packetSize + 1 : end);
        
        try
            % Parse the extracted byte vector directly back into 256 single-precision floats
            fftDataFrame = typecast(rawFrameBytes, 'single');
            
            % Refresh the active plot line smoothly
            set(fftCurve, 'YData', fftDataFrame);
            
        catch ME
            fprintf('Parsing warning: %s\n', ME.message);
        end
    end
    
    % Render visual updates fluidly without dropping interface interactions
    drawnow limitrate;
end

fprintf('Plot window terminated. Port cleared.\n');