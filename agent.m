classdef agent < handle
    % AGENT parent class   
    properties
        x % state (n-dim real vector, according to the dynamic model)
        f % model (object) 
        parameters % game parameters
        ID
        u % current control: struct with optional fields:
        % velocity (vector) OR
        % angle OR steering (normalized, [0,1]) AND OPTIONALLY
        % speed (=velocity norm) OR acceleration (normalized, [0,1])
        % the appropriate fields are used by the dynamics classes
    end
    
    methods
        function this = agent(model, parameters, ID)
            this.f = model;
            this.parameters = parameters;
            this.ID = ID;
        end
        
        function new_round(this, x0)
            this.x = x0;
        end

        function x = act(this)
            this.x = this.f.update(this.x, this.u, this.parameters.dt);
            x = this.x;
        end       
        
        function p = position(this)
            p = this.x(1:2);
        end
    end
    
    methods (Abstract)
        plan(this, XP, XE) % set this.u
    end
end

