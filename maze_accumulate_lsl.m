%function X = mr_accumulate_lsl (X)
function mr_accumulate_lsl
% reads real-time (or simulated real-time) data and accumulates it in the mr.accumulatedData field. 
%To save memory and prevent slowdown due to disk-swapping, the length of data accumulated in mr.accumulatedData can be limited automatically 
%in this function by setting the mr.maxNumberOfFramesInAccumulatedData field to a finite value. 
%
% 12/13 JRI change to use LSL
%   primarily, we need to ensure mr.accumulatedData is appended to
%   columnwise. It's structured [x1 y1 z1 x2 y2 z2 ...]'
% mr_show_mocap will format it into mocap markers
% global mr

global X;
if ~isfield(X.LSL,'phasespace'), error('phasespace LSL inlet must be initialized. There must have been a problem in mr_maze_with_lsl'), end

maximumDataToReadInEachRun = 2000; % maximum number of frames to be read in each acumulation run
counter = 1;
sizeOfBlockInceaseInAccumulatedDataSize = 30000;

%r=1;
%arr_in = ds_array; 

persistent n;

if isempty(n)
    n = 0;
end;

while  counter<maximumDataToReadInEachRun
    
    if X.readFromLSL, %JRI New, the only LSL specific code here
        [chunk, stamps] = X.LSL.phasespace.inlet.pull_chunk();
        r = length(stamps); %number of samples
        if r>0,
            arr_in.Event = [];
            %strip off 4th value for each marker (i think it's a valid/invalid
            %flag?
            chunk(4:4:end,:)=[];
            arr_in.nItems = size(chunk,1);
            arr_in.Data = chunk(:,end); %only take in the most recent positions
        end
    else
        [r,arr_in] = mr_read_simulated;
    end;
    
    if r>0
%         if mr.readFromDataRiver
%             % place newly received event in the 'event circular buffer'
%             mr.eventCircularBuffer = circular_buffer_add(mr.eventCircularBuffer, arr_in.Event);
%         end;
        
        % add new data to the accumulator variable 'mr.accumulatedData'
        X.numberOfChannelsInAccumulatedData = arr_in.nItems;
        
        if X.numberOfFramesInAccumulatedData+1 > size(X.accumulatedData)
            % expand the data if it has not resched the maximum allowed
            % size
            if X.numberOfFramesInAccumulatedData<X.maxNumberOfFramesInAccumulatedData                
               if X.verboseLevel > 0
                    fprintf('mr.accumulatedData expansion started...\n');
                end;
                X.accumulatedData = cat(2, X.accumulatedData, nan(size(X.accumulatedData,1), sizeOfBlockInceaseInAccumulatedDataSize));
                if X.verboseLevel > 0
                    fprintf('mr.accumulatedData expanded.\n');
                end;
            else % shrink accumulated data size if it has reached the max allowed size
                X.accumulatedData = cat(2, X.accumulatedData(:, (X.numberOfFramesInAccumulatedData - X.maxNumberOfFramesInAccumulatedData):X.numberOfFramesInAccumulatedData), nan(size(X.accumulatedData,1), sizeOfBlockInceaseInAccumulatedDataSize));
                X.numberOfFramesInAccumulatedData = X.maxNumberOfFramesInAccumulatedData;
                if X.verboseLevel > 0
                    fprintf('mr.accumulatedData shrunk.\n');
                end;
            end;
        end;
        
        X.accumulatedData(1:X.numberOfChannelsInAccumulatedData, X.numberOfFramesInAccumulatedData+1) = arr_in.Data(1:arr_in.nItems);
        X.numberOfFramesInAccumulatedData = X.numberOfFramesInAccumulatedData + 1;
        X.numberOfFramesReceived =  X.numberOfFramesReceived + 1;

        % DEM 9.30.2015
        % excerpted from mr_show_mocap.m -- this code belongs in here
        % honestly, I'm not sure why we need all this accumulate nonesense
        % it seems like we can just grab the last sample our of the chunk
        % directly and use that here
        if isnan(X.mocap.lastChannel) % nan for lastChannel means that all channels after firstChannel are mocap, this is useful for when the actual number is not know a priori
            mocapChannels = X.mocap.firstChannel : X.numberOfChannelsInAccumulatedData;
        else
            mocapChannels = X.mocap.firstChannel : X.mocap.lastChannel;
        end;

        data = X.accumulatedData(mocapChannels, X.numberOfFramesInAccumulatedData);
        channelOffset = 0;

        maxChan = channelOffset + floor((length(data) - channelOffset)/3) * 3;

        ys = double(data((1+channelOffset):3:maxChan));
        zs = double(data((2+channelOffset):3:maxChan));
        xs = double(data((3+channelOffset):3:maxChan));

        % JRI/MM call invalid channels those very close to 0,0,0
        invalidChannelId = find(abs(xs) < 0.002 | abs(ys) < 0.002 | abs(zs) < 0.002);
        invalidChannelId = union(invalidChannelId, find(isnan(xs)));

        % put invalid points really far away, for now
        zs(invalidChannelId) = -100;

    else
        break;
    end;
    counter = counter + 1;
end




% run the function given by handle after data is updated
if isa(X.functionHandle, 'function_handle')
    X.functionHandle();
end;

end