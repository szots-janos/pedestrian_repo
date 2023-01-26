function circle(obsteclesPositionsX,obsteclesPositionsY,obstaclesRadius,color,language,xlimits,ylimits)
% This function draw circles

% Input:
%  obsteclesPositionsX: x center point of obstacles, Vector: 1*nObstacles
%  obsteclesPositionsY: y center point of obstacles, Vector: 1*nObstacles
%  obstacleRadius: radii of obstacles, Vector: 1*nObstacles
%  color:  what colored will be the boundary of the circle
%  language: the language of the texts of the labels
%  xlimits: on the figures the boundaries of the x coordinates
%  ylimits: on the figures the boundaries of the y coordinates

nObstacles=length(obsteclesPositionsX); % number of obstacles
for iObstacle =1:nObstacles
    theta = 0 : 0.01 : 2*pi;
    x = obstaclesRadius(iObstacle) * cos(theta) + obsteclesPositionsX(iObstacle);
    y = obstaclesRadius(iObstacle) * sin(theta) + obsteclesPositionsY(iObstacle);
 %p2=patch('XData', x, 'YData', y, 'FaceColor', 'r', 'FaceAlpha', .3)
 
        plot(x, y,color);
        axis equal;
        xlim(xlimits);
        ylim(ylimits);
        if (language ==1)
            xlabel('x koordináták [m]')
            ylabel('y koordináták [m]')
        else
            xlabel('x coordinates [m]')
            ylabel('y coordinates [m]')
        end
        grid on
        hold on
    
end
end