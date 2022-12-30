classdef P_random < agent
    properties
        p % chance of direction change
    end
    
    methods
        function this = P_random(varargin) % base args: model, parameters, ID
            % specific args: direction change interval
            this=this@agent(varargin{1:3});
            args = varargin{4};
            this.p = this.parameters.dt / args.avg_time;
        end
        
        function new_round(this, x0)
            this.x = x0;
            this.u.angle = rand*2*pi;
        end
        
        function plan(this, ~, ~)    
            if rand < this.p
                this.u.angle = 2*pi*rand;
            end
        end
    end
end