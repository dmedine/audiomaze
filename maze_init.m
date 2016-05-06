% intialize the audio maze and peripherals (mocap, lsl audio control
% stream)

% maze_lines and maze_polygons are generated by a call to
% make_maze_polygons (for a random maze) or make_maze_polygons_nr (for a
% predetermined maze)

function mr = maze_init(maze_lines, n_rows, n_cols, h, w, doVR)


    if nargin < 4
        doVrPlot = false;
    elseif doVR == false 
        doVrPlot = false;
    else
        doVrPlot = true;
    end
    
    %clear mr;
    %global mr;
    mr = [];
    mr.readFromLSL = true; % ***
    mr.functionHandle = [];
    mr.samplingRate  = 512;%512;
    mr.maxNumberOfFramesInAccumulatedData = 6000;% Inf;
    
    mr.h = h;
    mr.w = w;
    
    % note: added 11/13/15 DEM
    % mechanism for dealing with emitter drop errors
    % store the last good set of hand emitters and replace 0s 
    % with these values

    %global lastHandMarkers;
    mr.lastHandMarkers = zeros(8,3);

    % mr_init;
    % from the script above:
    maxChannels = 100;
    initialLength = 10000;
    mr.verboseLevel = 1; % 0 is no verbosity, 1 for medium and 2 is for max
    mr.accumulatedData = nan(maxChannels, initialLength);
    mr.numberOfChannelsInAccumulatedData = nan;
    mr.numberOfFramesInAccumulatedData = 0;
    mr.numberOfFramesReceived = 0;
    mr.event = [];
    mr.eventChannelNumber = nan; % when reading from datariver, event channel is not represented as a separate channel, but as the .event field of incoming samples.
    mr.eeg.channelOffset = 0;

    mr.doVrPlot = doVrPlot;

    
    mr.overheads = [0,0]; % test, center of room
    mr.n_overheads = 1;
    mr.inTokenTol = .1; % 1/10m
    mr.outTokenTol = 1.5; % you must get this far away until you can replay
    delete(timerfindall);
    
    % variables for keeping track of wall touch stats in real-time
    mr.was_near_wall = 0;
    mr.time_near_wall = 0;
    mr.total_time_near_wall = 0;
    mr.was_in_wall = 0;
    mr.time_in_wall = 0;
    mr.total_time_in_wall = 0;
    
    mr.in_wall_cnt = 0;
    mr.near_wall_cnt = 0; % counters for wall touches

    mr.time_was = 0;
    
    mr.proximityDistanceThreshold = 0.3; %set the distance from wall at which hand proximity sounds will begin

    % determine the 'in wall' proximity threshold according to how max/msp
    % understands it (midi units, 0-127)
    % the proximity here is 0-1 (once we cross the near wall threshold) so
    % we need to map MAX/msp's notion of in the wall to the audiomaze
    % engine's notion
    MAX_wall_prox_thresh = 110; % got this from the MAX patch
    mr.in_wall_prox = MAX_wall_prox_thresh/127; 

    % buoy playback control
    mr.buoy_time_accum = 0;
    mr.buoy_time_thresh = [10 20]; % time in the cycle to sound beacon
    mr.buoy_trig = [1 1]; % trigger the sound on or off
    
    %makoto mr_init_writing('/tmp/AudioSuite', 10, 20); 

    mr.numberOfFramesInAccumulatedData = 0;

    % for mocap, specify mocap channel subset
    mr.mocap.firstChannel = 1; % first channel is events or should be ignored
    mr.mocap.lastChannel = nan; % use nan to make it until the last one that exist

    mr.mocap.doSimplePlot = true;

     
    %% make the maze


    % for the maze
    
    mr.am = audioMaze(mr.h, mr.w, n_rows, n_cols, maze_lines);

    figure(11);
    mr.am.plotMaze();
    hold on;
    
    
    %% vr world stuff

    if mr.doVrPlot == true;
        if isfield(mr, 'mocap') && isfield(mr.mocap, 'mocapWorld') && ~isempty(mr.mocap.mocapWorld)
            close(mr.mocap.mocapWorld);
            delete(mr.mocap.mocapWorld);
        end;
        cur_dir = pwd;
        vr_path = strcat(cur_dir,'\vr\minimal_with_axis_captions');
        mocapWorld = vrworld(vr_path, 'new');
        open(mocapWorld);

        mr.mocap.mocapWorld = mocapWorld;
        mr.mocap.roomWallCollection = vr_draw_maze(mr.mocap.mocapWorld, mr.am); 
        
        figureHandle = view(mr.mocap.mocapWorld);
        vrdrawnow;
   end

    %% initialize LSL, connect to MaxMSP (via patch lslreceive)
    addpath(genpath('C:\DEVEL\labstreaminglayer\LSL\liblsl-Matlab'));
    if isfield(mr,'LSL'), mr = rmfield(mr,'LSL'); end
    mr.LSL.lib = lsl_loadlib();

    % while we are at it, initiallize the current clock time
    mr.time_was = lsl_local_clock(mr.LSL.lib);
    %init outlets to MAX
    disp('Initializing LSL outputs to MAX/MSP')
    
    mr.LSL.MaxMSP.streamInfo(1) = lsl_streaminfo(mr.LSL.lib,'fileplay','AudioControl',6,0,'cf_string','fileplay_AudioControl');
    mr.LSL.MaxMSP.outlet(1) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(1));
    
    mr.LSL.MaxMSP.streamInfo(2) = lsl_streaminfo(mr.LSL.lib,'handproximity','AudioControl',3,0,'cf_string','handproximity_AudioControl');
    mr.LSL.MaxMSP.outlet(2) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(2));
    
    mr.LSL.MaxMSP.streamInfo(3) = lsl_streaminfo(mr.LSL.lib,'noisepitch','AudioControl',2,0,'cf_string','noisepitch_AudioControl');
    mr.LSL.MaxMSP.outlet(3) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(3));
   
    mr.LSL.MaxMSP.streamInfo(4) = lsl_streaminfo(mr.LSL.lib,'overhead','AudioControl',3,0,'cf_string','overhead_AudioControl');
    mr.LSL.MaxMSP.outlet(4) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(4));
    
    mr.LSL.MaxMSP.streamInfo(5) = lsl_streaminfo(mr.LSL.lib,'headwall','AudioControl',3,0,'cf_string','headwall_AudioControl');
    mr.LSL.MaxMSP.outlet(5) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(5));
    
    mr.LSL.MaxMSP.streamInfo(6) = lsl_streaminfo(mr.LSL.lib,'buoys','AudioControl',2,0,'cf_string','buoy_AudioControl');
    mr.LSL.MaxMSP.outlet(6) = lsl_outlet(mr.LSL.MaxMSP.streamInfo(6));
    % functions to play a beacon sound, or wall-proximity sound
    % using previous convention of first 6 values being commands to play beacon
    % sounds from a given azimuth. To control wall proximity sounds, the azimuth, scaled
    % proximity, and wall event code are sent as values 7-9. Consider revising this
    % in future.
    % proximityDistance is scaled from 1(at proximityDistance from wall to 0
    % (touching wall)
    %
    %NB: Max lslaudo receiver only takes 8 arguments and expects first argument
    %to be a .wav. We'll needed to rewrite this
    mr.LSL.MaxMSP.play_sound = @(beaconSoundID, soundOn, loop, azimuth, volume, beaconEventCode) ...
        mr.LSL.MaxMSP.outlet(1).push_sample({num2str(beaconSoundID), num2str(soundOn), num2str(loop), num2str(azimuth),...
        num2str(volume), beaconEventCode});
    mr.LSL.MaxMSP.send_hand_proximity = @(proximityDistance, proximityAzimuth, proximityEventCode) ...
        mr.LSL.MaxMSP.outlet(2).push_sample({num2str(proximityDistance), num2str(proximityAzimuth), proximityEventCode});
    mr.LSL.MaxMSP.send_noise_freq = @(pitch, fooEventCode) ...
        mr.LSL.MaxMSP.outlet(3).push_sample({num2str(pitch), fooEventCode});
    mr.LSL.MaxMSP.send_overhead = @(which, what, eventcode) ...
        mr.LSL.MaxMSP.outlet(4).push_sample({num2str(which), num2str(what), eventcode});
    mr.LSL.MaxMSP.send_headwall = @(proximityDistanceHead, proximityAzimuthHead, proximityEventCodeHead) ...
        mr.LSL.MaxMSP.outlet(5).push_sample({num2str(proximityDistanceHead), num2str(proximityAzimuthHead), proximityEventCodeHead});
      mr.LSL.MaxMSP.play_buoy = @(buoyCode, buoyEventCode) ...
        mr.LSL.MaxMSP.outlet(6).push_sample({num2str(buoyCode), buoyEventCode});

    %% init input from phasespace
    streaminfo = {};
    disp('Waiting for Mocap stream...')
    while isempty(streaminfo),
        streaminfo = lsl_resolve_byprop(mr.LSL.lib,'type','Mocap',1); % look for mocap device
        drawnow
    end
    disp('Found Mocap Stream')
    mr.LSL.phasespace.streamInfo = streaminfo{1};
    mr.LSL.phasespace.inlet = lsl_inlet(mr.LSL.phasespace.streamInfo);

    %% set up marker indexes
    %sensor numbers will depend on the phasespace profile used
    % these are for head and gloves
    %

    % %% "head and hands" phasespace configuration
    % if 0,
    %     mr.mocap.markers.phasespaceConfiguration = 'head and hands';
    %     mr.mocap.markers.head = 1:4;
    %     mr.mocap.markers.leftHand = 5:12;
    %     mr.mocap.markers.rightHand = 13:18;
    % end

    %% "Full Body 1 with DG (48)" configuration
    %mr.mocap.markers.phasespaceConfiguration = 'Full Body 1 with DG (48)';
    %mr.mocap.markers.head = [1:3 47];
    %mr.mocap.markers.rightHand = 11:18;
    %mr.mocap.markers.leftHand = 23:30;
    %%

    % "4 gloves, 2 heads (dev)" configureation
    mr.mocap.markers.phasespaceConfiguration = '4 gloves, 2 heads (dev)';
    mr.mocap.markers.head = 1:4;
    mr.mocap.markers.rightHand = 5:12;
    mr.mocap.markers.leftHand = 13:20;


end