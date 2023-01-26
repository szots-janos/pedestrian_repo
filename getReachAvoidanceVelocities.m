function [RAVx, RAVy]= getReachAvoidanceVelocities(robotPositionX,robotPositionY, RV, obstaclesPositionsX,obstaclesPositionsY, obstaclesRadius,...
    VO_x, VO_y,use_rules, drawingTest,language,xlimits,ylimits)
% This function calculates the Reachable Avoidance Velocities (RAV) set of the mobile robot

% Input:
% robotPositionX: position of the robot (x coordinate)
% robotPositionY: position of the robot (y coordinate)
% RV: the reachable velocities in a matrix, MATRIX:[2*4]
% obstaclesPositionsX: position of the obstacles (x coordinate), Vector: 1*nObstacles
% obstaclesPositionsY: position of the obstacles (y coordinate), Vector: 1*nObstacles
% obstacleRadius: radii of obstacles Vector: 1*nObstacles
% VO_x : boundary points of the VOs (x coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)
% VO_y : boundary points of the VOs (y coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)
% use_rules: which obstacle will influence the motion planning algorithm, Vector: 1*nObstaclesthe
% drawingTest:  figures will be drawn (1:yes, 0: no)
% language: 1 hungarian, other: english
% xlimits: on the figures the boundaries of the x coordinates
% ylimits: on the figures the boundaries of the y coordinates

% Output:
% RAVx: the x coordinates of reachable avoidance velocities (maybe long vector)
% RAVy: the y coordinates of reachable avoidance velocities (maybe long vector)

nObstacles = size(VO_x,2);
x_RV = RV(1,:);
y_RV = RV(2,:);
    RAVx = x_RV;
    RAVy = y_RV;


if(drawingTest)
    circle(obstaclesPositionsX,obstaclesPositionsY,obstaclesRadius,'k',language,xlimits,ylimits)
end
for iObstacle = 1:nObstacles
    if (use_rules(iObstacle))
        % ki kell nullázni, hogy az iterációk során ne maradjon bent más adat, amit felhasznál
        
        x_collisionVelocities = [VO_x(3,iObstacle) ,VO_x(2,iObstacle),VO_x(1,iObstacle),VO_x(3,iObstacle)];
        y_collisionVelocities = [VO_y(3,iObstacle),VO_y(2,iObstacle),VO_y(1,iObstacle),VO_y(3,iObstacle)];
        
        % plot the VO-s
        if(drawingTest)
            plot(x_RV,y_RV,'y')
            patch(x_RV,y_RV,'y');
            %plot(x_utk_seb,y_utk_seb,'b')
            % patch(x_utk_seb,y_utk_seb,'r','FaceColor',[0.737; 0.663; 0.682]);
            alpha(0.6)
            axis equal;
            xlim(xlimits);
            ylim(ylimits);
            if (language ==1)
                xlabel('x [m]')
                ylabel('y [m]')
            else
                xlabel('x [m]')
                ylabel('y [m]')
            end
            grid on
            hold on
        end

        if isempty(polybool('intersection', x_RV, y_RV, x_collisionVelocities, y_collisionVelocities))
            elerheto_sebx = x_RV;
            elerheto_seby = y_RV;
                
            
        else
        [elerheto_sebx, elerheto_seby] = polybool('subtraction', x_RV, y_RV, x_collisionVelocities, y_collisionVelocities);

        
        end
        %         if(iObstacle)
%             RV_save{iObstacle,1} = elerheto_sebx;
%             RV_save{iObstacle,2} = elerheto_seby;
%             RAV_save{iObstacle,1} = RAVx;
%             RAV_save{iObstacle,2} = RAVy;
%             save('Adatok')
%             
%         end
        if(iObstacle>1)
            [RAVx, RAVy] = polybool('intersection', elerheto_sebx,elerheto_seby,RAVx,RAVy);
        else
            RAVx = elerheto_sebx;
            RAVy = elerheto_seby;
        end
    end
end
%length(RAVx)
if drawingTest  % draw the RAV set
    circle(obstaclesPositionsX,obstaclesPositionsY,obstaclesRadius,'k',language,xlimits,ylimits)
    plot(robotPositionX,robotPositionY,'ro')
    % RAVx = RAVx(~isnan(RAVx)); % ezt valószínû ki kell venni, különben rossz eredmény lesz
    % RAVy = RAVy(~isnan(RAVy));
    patch(double(RAVx),double(RAVy),'y'); % sárgával a metszet
    patch([VO_x(1:3,find(use_rules==1));VO_x(1,find(use_rules==1))],[VO_y(1:3,find(use_rules==1));VO_y(1,find(use_rules==1))],'r','FaceColor',[0.737; 0.663; 0.682]);
    alpha(0.6)
    axis equal;
    % 
    if(isempty(RAVx))
        error("There are no velocity components in the RAV so the motion planning to the target is not feasible!")
    end
end

end