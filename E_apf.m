classdef E_apf < agent
    properties            
        target_p
        rho_mult
        accel_mult
        LOG % for debug purpose
    end
    
    methods
        function this = E_apf(varargin)
            this=this@agent(varargin{1:3});
            assert(this.f.name=="dubins")
            args = varargin{4};
            this.target_p = args.target_p;
            this.rho_mult = args.rho_mult;
            this.accel_mult = args.accel_mult;
        end
        function new_round(this,x0)
            this.x=x0;
            this.LOG=[];
        end
        function plan(this,XP,XE)        
            % force pulling towards the target
            ET=this.parameters.target-XE(1:2);
            ETn=norm(ET);
            F=ET/ETn*this.target_p;
            
            % add forces pushing away from obstacles
            rho=this.parameters.dc*this.rho_mult;
            distances=vecnorm(XP(1:2,:)-XE(1:2));
            for i=1:this.parameters.NP
                F=F+exp(-(distances(i)/rho)^2)*(XE(1:2)-XP(1:2,i));
            end
            
            % generate controls
            this.u.acceleration = [cos(XE(3)),sin(XE(3))]*F*this.accel_mult;
            this.u.angle = atan2(F(2),F(1));
            
            % debug data
            if this.parameters.runs == 1 % do not store debug data for multiple runs
                this.LOG=[this.LOG;this.u.angle,distances,this.u.acceleration,XE(4)];
            end
        end
        
    end
end

