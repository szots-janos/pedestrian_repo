function RV = getReachableVelocities(robotPositionX, robotPositionY, velx, vely, robotacc_backward, MAXrobotvelocity, velocityAngle)
% This function calculates the reachable velocities of the robot 

% Input:
% robotPositionX: position of the robot (x coordinate)
% robotPositionY: position of the robot (y coordinate)
% velx: velocity of the robot (x coordinate)
% vely: velocity of the robot (y coordinate)
% robotacc_backward: backward acceleration of the robot so the decelaration
% MAXrobotvelocity: the maximum speed of the robot
% velocityAngle: the angle of the reachable velocities

% Output:
% RV: the reachable velocities in a matrix [2*4] 
% 2 rows: 1. row: x coordinates, 2. row: y coordinates of the velocity
% components, the first and the last coordinate is the same

RV = [];
angleRobotVelocity = atan2(vely,velx);
lengthRobotVelocity = sqrt(velx^2+vely^2);

%% backward: 
% first component of reachable velocities:
        if((lengthRobotVelocity-robotacc_backward) < 0)
            elerhetosebesseg(1,1) = robotPositionX;
            elerhetosebesseg(2,1) = robotPositionY;
        else  % at the rules as an assumption the first component is always the position of the robot
            elerhetosebesseg(1,1) =  robotPositionX;%(lengthRobotVelocity-robotacc_backward)*cos(angleRobotVelocity)+robotPositionX;
            elerhetosebesseg(2,1) =  robotPositionY;%(lengthRobotVelocity-robotacc_backward)*sin(angleRobotVelocity)+robotPositionY;
        end
        
%% up-down:        
        r=MAXrobotvelocity/cos(velocityAngle);
        elerhetosebesseg(1,2) = (r+0.01)*cos((angleRobotVelocity+velocityAngle))+robotPositionX;   %down (to x angle)
        elerhetosebesseg(2,2) = (r+0.01)*sin((angleRobotVelocity+velocityAngle))+robotPositionY;
        elerhetosebesseg(1,3) = r*cos((angleRobotVelocity-velocityAngle))+robotPositionX;   %up (to y angle)
        elerhetosebesseg(2,3) = r*sin((angleRobotVelocity-velocityAngle))+robotPositionY;
  
       
% the first coordinate will be also at the end of the matrix (it's appropriate because of the drawing)
RV(1,:)= [elerhetosebesseg(1,:), elerhetosebesseg(1,1)]; 
RV(2,:) = [elerhetosebesseg(2,:),elerhetosebesseg(2,1)];
end
