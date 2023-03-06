classdef E_svo < agent
    properties
        XP_prev
        LOG % for debug purpose
        args
       
    end
    
    methods
        function this = E_svo(varargin)
            this=this@agent(varargin{1:3});
            args = varargin{4};
            this.args = args;
            
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
            
          %  aa = this.args.T_SVO;
          %  a = this.parameters.dc;
          %  u.acceleration = 1; % relative
          %  u.steering = 1;
          %  b = this.f.update(XE,u,this.parameters.dt);
            
            
            
            velx = XE(4)*cos(XE(3));
            vely = XE(4)*sin(XE(3));
            angleRobotVelocity = atan2(vely,velx);
            MAXrobotvelocity = this.parameters.vE;
            obstaclesPositionsX = XP(1,:);
            obstaclesPositionsY = XP(2,:);
            obstaclesVelocitiesX = pursuer_velocities(1,:);
            obstaclesVelocitiesY = pursuer_velocities(2,:);
            obstaclesRadius = ones(1,this.parameters.NP)*this.parameters.dc;
            robotRadius = 0;
            goalPositionX = this.parameters.target(1);
            goalPositionY = this.parameters.target(2);
            investigatedVelocityAngle = pi/6;
            T_svo = 1.4;
            robotacc_backward = 0;
            drawingTest = 0;
            language = 2;
            xlimits = [min(min(obstaclesPositionsX), goalPositionX), max(max(obstaclesPositionsX),goalPositionX)];
            ylimits = [min(min(obstaclesPositionsY),goalPositionY), max(max(obstaclesPositionsY),goalPositionY)];
            distancesRobotObstacles = distance(XE(1), XE(2), obstaclesPositionsX, obstaclesPositionsY );
            
            if (XE(1)==0 && XE(2)==0)
                use_rules = ones(1,this.parameters.NP);
            else
                [Tcpa,Dcpa] = robot_obstacle_contact(XE(1),XE(2),velx,vely,obstaclesPositionsX,obstaclesPositionsY,obstaclesVelocitiesX,obstaclesVelocitiesY);
                for iObstacle=1:this.parameters.NP
                  if( ((Tcpa(iObstacle)>0) &&(Tcpa(iObstacle)<10) && (Dcpa(iObstacle)<10)) || (distancesRobotObstacles(iObstacle)<15) )  %(3*maxsebessegnagysag*t)
                    use_rules(1,iObstacle) = 1;
                  else
                    use_rules(1,iObstacle) = 0;
                  end
                end
            end
          
%           if sum(use_rules)
%               MAXrobotvelocity = MAXrobotvelocity-2;
%           end
             % use_rules = ones(1,this.parameters.NP);
            
%             C_beta_distance = 0.2;
%             C_alfa = 0.8;
            if(min(distancesRobotObstacles)< 10)
            C_beta_distance = 0;
            C_alfa = 1;
            else
            C_beta_distance = 1;
            C_alfa = 0;
            end
%             
            [robotvelocity_x,robotvelocity_y] = getRobotVelocitySVOmethod(XE(1),...
            XE(2), velx, vely, MAXrobotvelocity,  obstaclesPositionsX, obstaclesPositionsY, obstaclesVelocitiesX, obstaclesVelocitiesY,...
            obstaclesRadius, robotRadius, goalPositionX, goalPositionY, investigatedVelocityAngle, T_svo, robotacc_backward, use_rules, drawingTest, language,xlimits,ylimits, C_beta_distance, C_alfa);
            
        velx = robotvelocity_x - XE(1);
        vely = robotvelocity_y - XE(2);
            % generate controls
            this.u.velocity = [velx; vely]; 
            
            % debug data
            if this.parameters.runs == 1 % do not store debug data for multiple runs
                this.LOG=[this.LOG;0];
            end
        end
        
    end
end

