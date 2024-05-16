classdef Policy < handle
    properties
        isSwungUp       % Flag to indicate if swing-up is completed
        isLQRReady      % Flag to indicate if LQR gains are computed
        K               % Feedback gain matrix
        targetX         % Target x position from configuration
    end

    methods
        function obj = Policy()
            obj.isSwungUp = false;
            obj.isLQRReady = false;
            obj.K = 0;
            % Load target x position from configuration
            config = INI('File', 'sys.ini');
            config.read();
            obj.targetX = config.get('StartPos').x;
        end
        
        function action = action(self, obs)
            sys = obs.agent;
            if ~self.isSwungUp
                % Compute the energy needed for swing-up
                desiredEnergy = sys.pole_weight * sys.g * sys.pole_h / 2;
                currentEnergy = 0.5 * sys.pole_I * sys.dtheta^2 + sys.pole_weight * sys.g * sys.pole_h * (-cos(sys.theta)) / 2;
                energyGap = desiredEnergy - currentEnergy;
                anglethre = 60;
                Kp = 1.5;
                if -sys.dtheta * cos(sys.theta) > 0
                    u = Kp * energyGap;
                else
                    u = Kp * -energyGap;
                end
                
                if abs(sys.theta - pi) < anglethre * pi / 180
                    self.isSwungUp = true;
                end
                
            else
                if ~self.isLQRReady
                    % Set the weights for the LQR
                    Q = sys.Cs' * sys.Cs;
                    Q(1,1) = 1;
                    Q(3,3) = 1;
                    R = 5;
                    
                    % Calculate LQR gain
                    self.K = lqr(sys.As, sys.Bs, Q, R);
                    self.isLQRReady = true;
                end
                
                % Compute control using LQR
                % adjust
                x = sys.x - self.targetX;
                adjust_theta = sys.theta - pi;

                state = [x; sys.dx; adjust_theta; sys.dtheta];
                u = -self.K * state;
            end
            action = u;
        end
    end
end
