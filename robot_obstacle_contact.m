 
function   [Tcpa,Dcpa] = robot_obstacle_contact(robotPositionX,robotPositionY,velx,vely,...
                         obstaclesPositionsX,obstaclesPositionsY,obstaclesVelocitiesX,obstaclesVelocitiesY)
% The function shows the time and distance when the robot and the obstacles
% will be at the nearest to each other

% Input::
% robotPositionX: position of the robot (x coordinate)
% robotPositionY: position of the robot (y coordinate)
% velx: velocity of the robot (x coordinate)
% vely: velocity of the robot (y coordinate)
% obstaclesPositionsX: position of the obstacles (x coordinate), Vector: 1*nObstacles
% obstaclesPositionsY: position of the obstacles (y coordinate), Vector: 1*nObstacles
% obstaclesVelocitiesX: velocity of the obstacles (x coordinate), Vector: 1*nObstacles
% obstaclesVelocitiesY: velocity of the obstacles (y coordinate), Vector: 1*nObstacles

% Output:
%   Tcpa: time when the robot will at the nearest point to obstacles, Vector: 1*nObstacles
%   Dcpa: distance when the robot will at the nearest point to obstacles, Vector: 1*nObstacles


nObstacles=size(obstaclesPositionsX,2);
        Tcpa=[];
        Dcpa = [];
    for iObstacle=1:nObstacles

        Num_Tcpa = (robotPositionX-obstaclesPositionsX(iObstacle))*(velx-obstaclesVelocitiesX(iObstacle))+(robotPositionY-obstaclesPositionsY(iObstacle))*(vely-obstaclesVelocitiesY(iObstacle));
        Den_Tcpa = (velx - obstaclesVelocitiesX(iObstacle))^2 + (vely - obstaclesVelocitiesY(iObstacle))^2;
        Tcpa(1,iObstacle) = -Num_Tcpa / Den_Tcpa;

        t= Tcpa(iObstacle);
        Dcpa_x = (robotPositionX + velx * t) - (obstaclesPositionsX(iObstacle) + obstaclesVelocitiesX(iObstacle) * t);
        Dcpa_y = (robotPositionY + vely * t) - (obstaclesPositionsY(iObstacle) + obstaclesVelocitiesY(iObstacle) * t);
        Dcpa(1,iObstacle) = sqrt(Dcpa_x^2 + Dcpa_y^2);
    end
end