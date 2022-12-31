classdef dubins < handle
    % 2D, min turn radius
    properties
        name='dubins'
        c_max % maximum curvature, 1 / m
        a_max % maximum acceleration, m / s^2
        v_max % maximum speed, m / s
        % state is stored in the agent: [x, y, phi, v]
    end
    
    methods
        function this = dubins(v_max,turn_radius,a_max)
            this.v_max=v_max;
            this.c_max=1/turn_radius;
            this.a_max=a_max;
        end
        
        function x_new = update(this,x,u,dt)
            % see agent class for details of control 'u'
            position = x(1:2);
            phi = x(3);
            v = x(4);
            
            % decompose target velocity input
            if isfield(u, 'velocity')
                u.speed = norm(u.velocity);
                u.angle = atan2(u.velocity(2), u.velocity(1));
            end
            
            % accelerate
            if isfield(u, 'acceleration')
                % direct acceleration control
                a = min(max(-1,u.acceleration),1) * this.a_max;
            elseif isfield(u, 'speed')
                % target speed
                a = min(max(-this.a_max, (u.speed - v) / dt), this.a_max);
            else
                % acceleration has default value
                a = 0; 
            end
            if abs(a) > eps
                v_new = min(max(0, v + a*dt), this.v_max);
                acceleration_time = (v_new - v) / a;
                distance = (v + v_new) / 2 * acceleration_time + v_new * (dt - acceleration_time);
            else
                v_new = v;
                distance = v_new * dt;
            end
            
            if isnan(distance), keyboard, end
            
            % turn
            if isfield(u, 'steering') 
                % direct steering control
                curvature = min(max(-1,u.steering),1) * this.c_max;
                % may be inf (straight movement)
                % negative value means right turn
            elseif isfield(u, 'angle')
                % target angle 
                target_diff = angle_diff(phi, u.angle);
                curvature = min(max(-this.c_max, target_diff / distance), this.c_max);
            else
                error('No steering input for dubins robot')
            end
            phi_new = phi + distance * curvature;
            
            % compute position
            if abs(curvature)<eps
                position_new = position + distance * [cos(phi); sin(phi)];
            else
                R = 1 / curvature;
                turn_center = position + R * [-sin(phi); cos(phi)];
                position_new = turn_center - R * [-sin(phi_new); cos(phi_new)]; 
            end
            
            x_new = [position_new; phi_new; v_new];
        end
    end
end

