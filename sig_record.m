%% STM32 Real-Time FFT Binary Serial Plotter (Non-Blocking Sync)
clear; clc; close all;

%% CONFIGURATION
portName = "COM8";       
baudRate = 2000000;
AUDIO_REC = 1024;
fftSize = AUDIO_REC / 2;          
numBins = fftSize / 2;            
binsToSend = fftSize / 2;                 % 128 floats expected per packet
fs = 16000;                       

% Calculate frequency resolution per bin
freqAxis = (0:binsToSend-1) * (fs / fftSize); 

%% SETUP GRAPHICS
fig = figure('Name', 'STM32 Live FFT Spectrum', 'NumberTitle', 'off');
ax = axes('Parent', fig);
grid(ax, 'on');
xlabel(ax, 'Frequency (Hz)');
ylabel(ax, 'Magnitude');
title(ax, 'Real-Time FFT Spectrum (Robust Non-Blocking Stream)');

fftCurve = plot(ax, freqAxis, zeros(1, binsToSend), 'Color', [0 0.5 0.8], 'LineWidth', 1.5);
xlim(ax, [0, freqAxis(end)]);
ylim(ax, [0, 100000000]); 

plotFig = [];

%% INITIALIZE SERIAL PORT
try
    device = serialport(portName, baudRate);
    flush(device);
    fprintf('Connected to %s. Initializing stream...\n', portName);
catch ME
    error('Could not connect. Ensure PuTTY is completely CLOSED! Info: %s', ME.message);
end

cleanupObj = onCleanup(@() delete(device));

%% MULTI-RUN RECORDING CONFIGURATION
maxRuns = 50;
currentRun = 1;
triggerCountdown = true; 
isRecording = false;
recordedData = [];

% Define the calibration sync header
SYNC_HEADER = [0xAA, 0xBB, 0xCC, 0xDD];
headerSize = length(SYNC_HEADER);
dataSizeInBytes = binsToSend * 4; % 128 floats * 4 bytes = 512 bytes
packetSize = dataSizeInBytes + headerSize; % 516 bytes total

% Persistent internal stream buffer to prevent hanging
serialBuffer = uint8([]); 

%% MAIN PROCESSING LOOP
while ishandle(fig) && (currentRun <= maxRuns)
    
    % --- SEQUENCE CONTROL: COUNTDOWN FOR NEXT RUN ---
    if triggerCountdown
        fprintf('\n--- [RUN %d/%d] Recording Initialization ---\n', currentRun, maxRuns);
        for countdown = 2:-1:1
            fprintf('%d...\n', countdown);
            pause(1);
        end
        fprintf('🔴 RECORDING STARTED (RUN %d)!\n\n', currentRun);
        
        recordedData = [];
        flush(device);     
        serialBuffer = uint8([]); % Clear local buffer
        tic;               
        isRecording = true;
        triggerCountdown = false;
    end

    % --- NON-BLOCKING DATA ACQUISITION ---
    bytesAvailable = device.NumBytesAvailable;
    if bytesAvailable > 0
        % Instantly read whatever raw bytes are in the hardware buffer without blocking
        rawData = read(device, bytesAvailable, "uint8");
        serialBuffer = [serialBuffer, rawData]; % Append to our local accumulation buffer
    end
    
    % Process the accumulated buffer only if we have at least one full potential packet
    while length(serialBuffer) >= packetSize
        
        % Search for the start position of the SYNC_HEADER pattern in our buffer
        headerIdx = strfind(serialBuffer, SYNC_HEADER);
        
        if isempty(headerIdx)
            % Header pattern not found at all. Keep only the last (headerSize - 1) bytes 
            % just in case a header was split across this chunk and the next incoming chunk.
            if length(serialBuffer) >= headerSize
                serialBuffer = serialBuffer(end - headerSize + 2 : end);
            end
            break; % Exit buffer processing loop and wait for more serial data
        end
        
        % If the first detected header is not at the very beginning (index 1),
        % drop the stale leading garbage data bytes before it.
        if headerIdx(1) > 1
            serialBuffer = serialBuffer(headerIdx(1):end);
            continue; % Re-evaluate loop with the header shifted to index 1
        end
        
        % Now we are guaranteed that serialBuffer starts exactly with our SYNC_HEADER.
        % Check if the rest of the complete data payload has arrived yet.
        if length(serialBuffer) < packetSize
            break; % Not enough data bytes yet for the full FFT frame. Wait for next loop iteration.
        end
        
        % Extract the 512 data bytes immediately following the 4-byte header
        rawFrameBytes = serialBuffer(headerSize + 1 : packetSize);
        
        % Safely pop the processed packet out of the buffer array
        serialBuffer = serialBuffer(packetSize + 1 : end);
        
        try
            % Convert the extracted byte sequence back into 128 single-precision floats
            fftDataFrame = typecast(rawFrameBytes, 'single');
            
            % Update graph visual properties
            set(fftCurve, 'YData', fftDataFrame);
            
            % --- RECORDING LOGIC ---
            if isRecording
                currentFrame256 = fftDataFrame(:);
                recordedData = [recordedData, currentFrame256];
                
                if toc >= 1.0
                    isRecording = false;
                    fprintf('⏹️  RECORDING %d ENDED! Saving file...\n', currentRun);

                    % Force the matrix to be exactly 256x16 (Truncate extra columns or pad with zeros)
                    recordedData = [recordedData(:, 1:min(end, 16)), zeros(256, max(0, 16 - size(recordedData, 2)))];
                    
                    fileName = sprintf('stm32_fft_record_snap_%d.csv', currentRun);
                    writematrix(recordedData, fileName);
                    fprintf('Saved: "%s" (Size: %dx%d)\n', fileName, size(recordedData, 1), size(recordedData, 2));
                    
                    % 2. Dynamic Window Management (Update or Create)
                    if isempty(plotFig) || ~ishandle(plotFig)
                        % Create the window for the very first time
                        plotFig = figure('Name', 'Latest Recorded Data Line Plot', 'NumberTitle', 'off');
                    else
                        % Target and clear the existing window instead of making a new one
                        figure(plotFig); 
                        clf(plotFig);
                    end
                    
                    % 3. Plot the recorded result in 2D
                    plot(recordedData, 'LineWidth', 1.5);
                    grid on;
                    
                    % 4. Add Labels and Styling
                    xlabel('Bins (1 to 256)', 'FontSize', 12, 'FontWeight', 'bold');
                    ylabel('Magnitude', 'FontSize', 12, 'FontWeight', 'bold');
                    title(sprintf('Recorded Frames Overlay (Run %d)', currentRun), 'FontSize', 14, 'FontWeight', 'bold');

                    currentRun = currentRun + 1;
                    if currentRun <= maxRuns
                        triggerCountdown = true; 
                    else
                        fprintf('\n🎉 All %d recording sessions completed successfully!\n', maxRuns);
                    end
                end
            end
            
        catch ME
            % Safeguard against accidental conversion discrepancies
            fprintf('Parsing warning: %s\n', ME.message);
        end
    end
    
    % Yield control to MATLAB graphics thread to maintain window fluidity
    drawnow limitrate;
end

if currentRun > maxRuns
    fprintf('All loops completed. Script finished.\n');
else
    fprintf('Plot window terminated. Port cleared.\n');
end