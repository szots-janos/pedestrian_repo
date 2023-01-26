function [robotvelocity_x,robotvelocity_y] = getRobotVelocitySVO(r, act_robotPositionX, act_robotPositionY, ...
    investigatedVelocityAngle, angleRobotVelocity, goalPositionX, goalPositionY, MAXrobotvelocity, C_beta_distance, C_alfa, VO_x, VO_y, T_svo, RAVx, RAVy, drawingTest,use_rules)


% investigate_velocities_x = act_robotPositionX+ (r*cos([1 0.5  0 -0.5 -1 ]*(investigatedVelocityAngle-0.01)+angleRobotVelocity))'*(0.2:0.2:1);
% investigate_velocities_y = act_robotPositionY+ (r*sin([1 0.5  0 -0.5 -1]*(investigatedVelocityAngle-0.01)+angleRobotVelocity))'*(0.2:0.2:1); % itt kivettem a sebessegszog ut�ni +0.01-et

investigate_velocities_x = act_robotPositionX+ (r*cos(linspace(-1, 1, 100)*(investigatedVelocityAngle-0.01)+angleRobotVelocity))'*(0.2:0.2:1);
investigate_velocities_y = act_robotPositionY+ (r*sin(linspace(-1, 1, 100)*(investigatedVelocityAngle-0.01)+angleRobotVelocity))'*(0.2:0.2:1); % itt kivettem a sebessegszog ut�ni +0.01-et

if drawingTest
    plot(investigate_velocities_x, investigate_velocities_y,'kx')
    hold on
end

investigate_velocities_x = reshape(investigate_velocities_x,numel(investigate_velocities_x),1 );
investigate_velocities_y = reshape(investigate_velocities_y,numel(investigate_velocities_y),1 );

% Cakculate the TG value:
investigated_velocities_angles = atan2((investigate_velocities_y - act_robotPositionY),(investigate_velocities_x - act_robotPositionX));
goal_Angle = atan2((goalPositionY - act_robotPositionY), (goalPositionX - act_robotPositionX));
angles_difference_from_goal = abs(investigated_velocities_angles - goal_Angle);
r_investigate_velocities = distance(investigate_velocities_x, investigate_velocities_y, act_robotPositionX, act_robotPositionY);
TD_value = (r_investigate_velocities.*cos(angles_difference_from_goal))./MAXrobotvelocity;
TD = TD_value * C_beta_distance;

safetyValue = vo_tav_different_alfas3(VO_x,VO_y, investigate_velocities_x,investigate_velocities_y,MAXrobotvelocity,T_svo);
Safety = safetyValue * C_alfa;

valueOfCostfunction = Safety + TD;

if (isempty(RAVx) && isempty(RAVy) )
    robotvelocity_x = act_robotPositionX;
    robotvelocity_y = act_robotPositionY;
else
    
    
    
    % new method for checking, wheather the velocity is in RAV or not:
    in = ones(length(investigate_velocities_x),1);
    for iVel=1:length(investigate_velocities_x)
        for iVO=1:length(VO_x)
            if(use_rules(iVO))
            if (in(iVel)==1 && (inpolygon(investigate_velocities_x(iVel),investigate_velocities_y(iVel),[VO_x(1:3,iVO);VO_x(1,iVO)], [VO_y(1:3,iVO);VO_y(1,iVO)])==0)) 
                in(iVel) = 1;
            else
                in(iVel) = 0;
            end
            end
        end
    end
    
    
    
    
   % in2 = inpolygon(investigate_velocities_x,investigate_velocities_y,RAVx, RAVy);
    valueOfCostfunction(find(in==0)) = 0;
    id = find(valueOfCostfunction==max(valueOfCostfunction));
    robotvelocity_x = investigate_velocities_x(max(id));
    robotvelocity_y = investigate_velocities_y(max(id));
    %Best_cost_GridSearch = valueOfCostfunction(id);
    success = 1;
end


%   plot(investigate_velocities_x,investigate_velocities_y,'rx')
%hold on


end