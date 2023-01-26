function [robotvelocity_x,robotvelocity_y] = getRobotVelocitySVOmethod(act_robotPositionX, act_robotPositionY, velx, vely, MAXrobotvelocity,  obstaclesPositionsX, obstaclesPositionsY, obstaclesVelocitiesX, obstaclesVelocitiesY, obstaclesRadius, robotRadius, goalPositionX, goalPositionY, investigatedVelocityAngle, T_svo, robotacc_backward, use_rules, drawingTest, language,xlimits,ylimits, C_beta_distance, C_alfa)



angleRobotVelocity = atan2(vely,velx);
obstaclesRadius = obstaclesRadius + robotRadius;
r = MAXrobotvelocity;
[tangentialPointsX,tangentialPointsY,wrongPos] = getTangentialPointsToObstacles(act_robotPositionX, act_robotPositionY,...
    obstaclesPositionsX, obstaclesPositionsY, obstaclesRadius);

if wrongPos
    robotvelocity_x = velx;
    robotvelocity_y = vely;
else
[VO_x,VO_y]= getVOs(act_robotPositionX,act_robotPositionY, MAXrobotvelocity, obstaclesVelocitiesX,obstaclesVelocitiesY,...
                tangentialPointsX,tangentialPointsY);
            
RV = getReachableVelocities(act_robotPositionX, act_robotPositionY, velx, vely, robotacc_backward, MAXrobotvelocity, investigatedVelocityAngle);
 

%[RAVx, RAVy]= getReachAvoidanceVelocities(act_robotPositionX,act_robotPositionY, RV, obstaclesPositionsX,obstaclesPositionsY, obstaclesRadius,...
 %               VO_x, VO_y,use_rules, drawingTest,language,xlimits,ylimits);
   RAVx = RV(1,:);
   RAVy = RV(2,:);
            
[robotvelocity_x,robotvelocity_y] = getRobotVelocitySVO(r, act_robotPositionX, act_robotPositionY, ...
          investigatedVelocityAngle, angleRobotVelocity, goalPositionX, goalPositionY, MAXrobotvelocity, C_beta_distance, C_alfa, VO_x, VO_y, T_svo, RAVx, RAVy, drawingTest,use_rules);
      
      if(drawingTest)
          hold on
      plot(robotvelocity_x,robotvelocity_y,'bo')
      end
end     
end