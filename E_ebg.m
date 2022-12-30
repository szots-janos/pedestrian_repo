classdef E_ebg < agent
    properties
        w % pedestrian speed
        reaction_distance % pursuers with miss distance above this are neglected
        focus_distance % if any miss distance is below, target is neglected
        safety_distance % if any miss distance is below, car brakes
        LOG % for debug purpose
    end
    
    methods
        function this = E_ebg(varargin)
            this = this@agent(varargin{1:3});
            assert(this.f.name == "dubins")
            args = varargin{4};
            this.reaction_distance = args.reaction_distance;
            this.focus_distance = args.focus_distance;
            this.safety_distance = args.safety_distance;
            this.w = max(this.parameters.vP, this.parameters.vE/2);
        end
        function new_round(this,x0)
            this.x=x0;
            this.LOG=[];
        end
        function plan(this,XP,XE)
            % variable names will correspond with the article except the 3
            % parameters
            % base variables
            dt = this.parameters.dt;
            a_max = this.parameters.aE;
            v = XE(4);
            phi = XE(3);
            % using cos(pi/2-phi) = sin(phi) and sin(phi/2-phi) = cos(phi)
            x = [sin(phi),-cos(phi)]*(XP(1:2,:)-XE(1:2));
            y = [cos(phi),sin(phi)]*(XP(1:2,:)-XE(1:2));
            
            % path measures
            T = v / a_max;
            S = v^2 / a_max / 2;
            d = vecnorm([x;y-S]);
            V = d - this.w * T;
            
            % maximum possible acceleration and resulting Value (miss distance)
            u_v_max = min(1, (this.parameters.vE - v) / a_max / dt);
            % decomposing deltaV = deltaV_phi * u_phi + deltaV_v * u_v + deltaV_independent
            deltaV_phi = S * x * v / this.parameters.rE * dt ./ d; % [m]
            deltaV_v = ((S - y) ./ d * v - this.w) * dt; % [m]
            V_acc = V + deltaV_v * (1 + u_v_max);
            
            if any(V_acc < this.safety_distance)
                % critical case
                this.u.acceleration = min((this.safety_distance-V)./deltaV_v - 1);
                this.u.steering = 0;
            else      
                % target conditions
                psi_target = atan2(this.parameters.target(2)-XE(2), this.parameters.target(1)-XE(1));
                V_N_1 = this.reaction_distance-angle_diff(phi,psi_target)/pi*(this.reaction_distance-this.focus_distance);
                deltaV_phi_N_1 = (this.reaction_distance-this.focus_distance)/pi*v / this.parameters.rE* dt; 
                V_N_2 = this.reaction_distance+angle_diff(phi,psi_target)/pi*(this.reaction_distance-this.focus_distance);
                deltaV_phi_N_2 = -(this.reaction_distance-this.focus_distance)/pi*v / this.parameters.rE * dt;
                
                % lin. program: max_u xi s.t. xi <= V_acc + deltaV_phi * u
                %                             xi <= V_N_1 + deltaV_phi_N_1 * u
                %                             xi <= V_N_2 + deltaV_phi_N_2 * u
                % -> xi <= A + B * u
                A = [V_acc, V_N_1, V_N_2];
                B = [deltaV_phi, deltaV_phi_N_1, deltaV_phi_N_2];
                % 1-variable linear program is trivial to solve:
                [~,idx]=min(A);
                nidx=[1:idx-1,idx+1:this.parameters.NP+2];
                dir=sign(B(idx));
                u_phi=-dir*(A(nidx)-A(idx))./(B(nidx)-B(idx));
                if all(u_phi<=0) || dir==0
                    this.u.steering=dir; % unconstrained case, maximum steering
                else
                    this.u.steering=dir*min(u_phi(u_phi>=0)); 
                end
                this.u.acceleration = u_v_max;
            end
            
            % debug data
            if this.parameters.runs == 1
                this.LOG=[this.LOG;this.u.steering,V_acc,this.u.acceleration,v,V];
            end
        end
    end
end

