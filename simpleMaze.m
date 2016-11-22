% function for initiating a simple maze session
% @param which    : upper case character A-F indicating which maze to build
% @param trial_no : which run through the maze (1-3)

function simpleMaze()

global which_maze;
%global random_seed;
global trial_number;
global X;
global mazeStopped;


condition = 1;
is_practice = 0;
while(condition)
    str = whichMazePrompt();
    if str == 'A' | str == 'B' | str == 'C' | str == 'D' | str == 'E' | str == 'F' | str == 'P'
        which_maze = str;
        condition = 0;
        
        % set the maze to 'A' for the practice round
        if which_maze == 'P'
            which_maze = 'A'
            is_practice = 1;
        end
    else
        disp('invalid maze selection, please choose A-F or P');
        condition = 1;
    end
end

if is_practice ~= 1
    condition = 1
    while(condition)
        
        trial = whichTrialPrompt();
        if trial >= 1 && trial <=4
            trial_number = trial;
            condition = 0;
        else
            disp('invalid trial selection, please choose 1-3');
            condition = 1;
        end
    end
    
    condition = 1
    while (condition)
        str = bonusPrompt();
        if str == 'y' | str == 'Y'
            f = fopen('bonusFile', 'W');
            fprintf(f, '%f', 0.0);
            fclose(f);
            condition = 0;
        else
            condition = 0;
        end
    end
    
    disp(sprintf('Initiating maze %s for trial %d', which_maze, trial_number));
    simpleInitScrpt();
    


    prompt = 'Maze initiated. Press any key to begin.\n';
    str = input(prompt,'s');
    if isempty(str)
        str = 'Y';
    end
    
    infoData = [which_maze, num2str(random_seed), num2str(trial_number)];
    X.LSL.emitInfo(which_maze, random_seed, trial_number);
    
    simpleTaskMainLoop;

% special case of practice maze
else
    disp(sprintf('Initiating practice maze maze %s', which_maze));
    simpleInitScrpt();
    
    prompt = 'Maze initiated. Press any key to begin.\n';
    str = input(prompt,'s');
    if isempty(str)
        str = 'Y';
    end
    simpleTaskMainLoop;

end

end

