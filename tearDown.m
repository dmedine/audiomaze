global X
global trial_number
global which_maze

disp(sprintf('Trial %d on maze %s ended',trial_number, which_maze));
%fId = fopen('velocityFile', 'r');
%formatSpec = '%f';
%vel = fscanf(fId,formatSpec);

% get the velocity
if ~isempty(X.velocityFile)
    fclose(X.velocityFile);
end
vel = csvread('velocityFile');
meanVel = mean(vel)



% emit stats here
statData = [meanVel, X.nearWallCntHand, X.touchingWallCntHand, X.totalTimeNearWallHand, X.totalTimeTouchingWallHand, X.nearWallCntHead, X.touchingWallCntHead, X.totalTimeNearWallHead, X.totalTimeTouchingWallHead];
X.LSL.emitStats(statData);

% prompt the maze exit
condition = 1;
while(condition==1)
    str = endPrompt();
    % stop and clear the maze
    if str == 'Y' | str == 'y'
        condition = 0;
        

        delete(timerfindall)
%        pause(2);
        close(X.am.fig_handle);
        
        % get the bonus
        disp(sprintf('current bonus: %1.2f',X.bonus));
        
        fId=fopen('bonusFile', 'r');
        formatSpec = '%f';
        totalBonus = fscanf(fId,formatSpec);
        totalBonus = totalBonus+X.bonus;
        fclose(fId);
        fId = fopen('bonusFile', 'W');
        fprintf(fId, '%f', totalBonus);
        fclose(fId);
        
        disp(sprintf('total bonus: %1.2f',totalBonus));
        clear all;
        break;
    else
    end
end
