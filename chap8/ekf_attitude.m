% ekf_attitude
% simple ekf for estimating roll and pitch
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef ekf_attitude < handle
   %--------------------------------
    properties
        Q
        Q_gyro
        R_accel
        N
        xhat
        P
        Ts
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ekf_attitude(SENSOR, SIM)
            % load SIM and SENSOR parameters
            run('../parameters/simulation_parameters')
            run('../parameters/sensor_parameters')
            self.Q = [0.0000001, 0; 0, 0.0000001]; %------------- incompleted 
            self.Q_gyro = [0.0000001, 0; 0, 0.0000001];
            self.R_accel = SENSOR.accel_sigma^2; 
            self.N =  10; % number of prediction step per sample
            self.xhat = [0,0.1030];  % initial state: phi, theta
            self.P = 0; %assimed form the other file
            self.Ts = SIM.ts_control/self.N;
        end
        %------methods-----------
        function state = update(self, state, measurement)
            self.propagate_model(state);
            self.measurement_update(state, measurement);
            state.phi = self.xhat(1);
            state.theta = self.xhat(2);
        end
        function f_ = f(self, x, state)

            phi = x(1);
            theta = x(2); 
            f_ = [state.p + state.q*sin(phi)*tan(theta) + state.r*cos(phi)*tan(theta); 
                state.q*cos(phi)-state.r*sin(phi)];
        end
        function h_ = h(self, x, state)
            phi = x(1);
            theta = x(2); 
            
            h_ = [state.q*state.Va*sin(theta) + 9.81*sin(theta); 
                -state.p*state.Va*sin(theta) - 9.81*cos(theta)*sin(phi) + state.r*state.Va*cos(theta); %---------INCOMPLETE
                -state.q*state.Va*cos(theta) - 9.81*cos(theta)*cos(phi)];
        end
        function self = propagate_model(self, state)
            % model propagation
            for i=1:self.N
                % propagate model
                self.xhat = self.xhat + self.f(self.xhat, state)*self.Ts;
                % compute Jacobian
                A = self.jacobian(@self.f, self.xhat, state);
                % compute G matrix for gyro noise
                G = 0; 
                % convert to discrete time models
                A_d = [1, 0; 0, 1] + A*self.Ts +A^2 * self.Ts^2;
                G_d = 0; 
                % update P with discrete time model
                self.P = A_d*self.P*A_d' + self.Ts^2*self.Q; %;%+ A*self.Ts + A^2*self.Ts^2;
            end
        end
        function self = measurement_update(self, state, measurement)
        % measurement updates
            threshold = 2.0;
            h = self.h(self.xhat, state);
            C = self.jacobian(@self.h, self.xhat, state);
            y = [measurement.accel_x; measurement.accel_y; measurement.accel_z];
            for i=1:3
                if abs(y(i)-h(i)) < threshold
                    Ci = C(i, :); 
                    L = (self.P * Ci')/(self.R_accel + Ci*self.P*Ci'); 
                    I = [1, 0;0 , 1]; 
                    self.P = (I-L*Ci)*self.P*(I-L*Ci)' + L*self.R_accel*L'; 
                    self.xhat = self.xhat + L*(y(i)-h(i));
                end
            end
        end
        function J = jacobian(self, fun, x, state)
            % compute jacobian of fun with respect to x
            f = fun(x, state);
            m = size(f, 1);
            n = size(x, 1);
            eps = 0.01;  % deviation
            J = zeros(m, n);
            for i=1:n
                x_eps = x;
                x_eps(i) = x_eps(i) + eps;
                f_eps = fun(x_eps, state);
                df = (f_eps - f) / eps;
                J(:, i) = df;
            end
        end
    end
end