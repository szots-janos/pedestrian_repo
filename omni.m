classdef omni < handle
    % 2D, instant turn, fixed velocity
    properties
        v % distance / timestep
        name='omni'
    end
    
    methods
        function this = omni(v, ~, ~)
            this.v=v;
        end
        
        function x_new = update(this,x,u,dt)
            if isfield(u, 'angle')
                x_new=x+this.v*[cos(u.angle);sin(u.angle)]*dt;
            else
                error('Omnidirectional robots require angle control')
            end
        end
    end
end

