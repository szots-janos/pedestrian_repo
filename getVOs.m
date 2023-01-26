function  [VO_x,VO_y]= getVOs(robotPositionX,robotPositionY, MAXrobotvelocity, obstaclesVelocitiesX,obstaclesVelocitiesY,...
                       tangentialPointsX,tangentialPointsY)
% In this function the VO-s will be calculated [4*nObstacles]

% Input:
% robotPositionX: position of the robot (x coordinate)
% robotPositionY: position of the robot (y coordinate)
% MAXrobotvelocity: the maximum speed of the robot
% obstaclesVelocitiesX: velocity of the obstacles (x coordinate), Vector: 1*nObstacles
% obstaclesVelocitiesY: velocity of the obstacles (y coordinate), Vector: 1*nObstacles
% tangentialPointsX : tangential points of the obstacles in x coordinates, MATRIX: 2*nObstacles
% tangentialPointsY : tangential points of the obstacles in y coordinates, MATRIX: 2*nObstacles

% Output:
%  VO_x : boundary points of the VOs (x coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)
%  VO_y : boundary points of the VOs (y coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)


VOdistance = 5 * MAXrobotvelocity;
% For every tangential point the angle will be calculated from the robot:
angle = atan2((tangentialPointsY - robotPositionY),(tangentialPointsX - robotPositionX));
VO_x(1:2,:) = robotPositionX + VOdistance * cos(angle) + ones(2,1) * obstaclesVelocitiesX;
VO_x(3:4,:) = ones(2,1) * obstaclesVelocitiesX + robotPositionX;

VO_y(1:2,:) = robotPositionY + VOdistance * sin(angle) + ones(2,1)*obstaclesVelocitiesY;
VO_y(3:4,:) = ones(2,1) * obstaclesVelocitiesY + robotPositionY;
end
