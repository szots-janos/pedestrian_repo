classdef E_svo < agent
    properties
        XP_prev
        LOG % for debug purpose
    end
    
    methods
        function this = E_svo(varargin)
            this=this@agent(varargin{1:3});
            args = varargin{4};
        end
        function new_round(this,x0)
            this.x=x0;
            this.XP_prev = [];
            this.LOG=[];
        end
        function plan(this,XP,XE)        
            % estimate pursuer velocities
            if size(XP, 1) == 4
                % pursuers are dubins vehicles, x(3) is angle, x(4) is speed
                pursuer_velocities = XP(4, :) * [cos(XP(3, :)); sin(XP(3, :))];
            else % size(XP, 1) == 2
                % pursuers are omnidirectional, velocity is estimated
                if isempty(this.XP_prev)
                    pursuer_velocities = zeros(2, this.parameters.NP);
                else
                    pursuer_velocities = (XP - this.XP_prev) / this.parameters.dt;
                end
                this.XP_prev = XP;
            end
            
            % SVO
            % input: evader state, XE, 4x1 vector: x, y, phi, v
            %        pursuer states XP, 2xNP or 4xNP vector, 
            %                                   where XP(1:2, :) are positions
            %        pursuer_velocities 2xNP matrix of (x,y) velocities
            % output: this.u.velocity, 2x1 vector, (x, y) m/s 
            
            % ALGORITHM HERE
            
            % generate controls
            this.u.velocity = [-3; 2]; 
            
            % debug data
            if this.parameters.runs == 1 % do not store debug data for multiple runs
                this.LOG=[this.LOG;0];
            end
        end
        
    end
end

