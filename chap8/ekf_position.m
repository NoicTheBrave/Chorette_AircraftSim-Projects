% ekf_position
% simple ekf for estimating pn, pe, chi, Vg, wn, we, psi
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/4/2019 - RWB
classdef ekf_position < handle
   %--------------------------------
    properties
        Q
        R
        N
        xhat
        P
        Ts
        gps_n_old
        gps_e_old
        gps_Vg_old
        gps_course_old
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ekf_position(SENSOR, SIM)
            % load SIM and SENSOR parameters
            run('../parameters/simulation_parameters')
            run('../parameters/sensor_parameters')
            self.Q = [
            0.00000000001,0,0,0,0,0,0;
            0,0.00000000001,0,0,0,0,0;  
            0,0,0.00000000001,0,0,0,0;
            0,0,0,0.00000000001,0,0,0;
            0,0,0,0,0.00000000001,0,0;
            0,0,0,0,0,0.00000000001,0;
            0,0,0,0,0,0,0.00000000001]; %there is like 7 lines of data

            self.R = [SENSOR.gps_n_sigma^2; 
                SENSOR.gps_e_sigma^2; 
                SENSOR.gps_Vg_sigma^2; 
                SENSOR.gps_course_sigma^2; 
                ]; 
            self.N = 10;  % number of prediction step per sample
            self.Ts = (SIM.ts_control / self.N);
            self.xhat = [0;0;20;0;0;0;0];
            self.P = 0; 
            self.gps_n_old = 9999;
            self.gps_e_old = 9999;
            self.gps_Vg_old = 9999;
            self.gps_course_old = 9999;
        end
        %------methods-----------
        function state = update(self, state, measurement)
            self.propagate_model(state);
            self.measurement_update(state, measurement);
            state.pn = self.xhat(1);
            state.pe = self.xhat(2);
            state.Vg = self.xhat(3);
            state.chi = self.xhat(4);
            state.wn = self.xhat(5);
            state.we = self.xhat(6);
            state.psi = self.xhat(7);
        end
        function f_ = f(self, x, state)
            pn = x(1); 
            pe = x(2);
            Vg = self.xhat(3);
            chi = self.xhat(4);
            wn = self.xhat(5);
            we = self.xhat(6);
            psi = x(7); 

            f_ = [ Vg*cos(chi); 
                    Vg*sin(chi); 
                    ((state.Va*cos(psi) + wn)*(-state.Va*psi*sin(psi))+(state.Va*sin(psi)+we)*state.Va*psi*cos(psi))/Vg;
                    9.81/Vg*tan(state.phi); 
                    0;
                    0;
                   state.q*sin(state.phi)/cos(state.theta) + state.r*cos(state.phi)/cos(state.theta) ];%---------Incompleted
                    
        end
        function h = h_gps(self, x, state)
            pn = x(1); 
            pe = x(2);
            Vg = self.xhat(3);
            chi = self.xhat(4);
            wn = self.xhat(5);
            we = self.xhat(6);
            %pe = self.xhat(2);
            psi = x(7); 
            h = [pn;
                pe; 
                Vg; 
                chi; 
                state.Va*cos(psi)+wn-Vg*cos(chi); 
                state.Va*sin(psi)+we-Vg*sin(chi)];
        end
%        function h = h_pseudo(self, x, state)
            % measurement model for wind triangle pseudo measuremnt
 %           h = 
  %     end
        function self = propagate_model(self, state)
            % model propagation
            for i=1:self.N
                % propagate model
                self.xhat = self.xhat + self.f(self.xhat,state)*self.Ts; %----------Incomplete
                % compute Jacobian
                A = self.jacobian(@self.f, self.xhat, state);
                % update P with continuous time model
                % self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
                % convert to discrete time models
                I = [ %Identity matrix
                        1, 0, 0, 0, 0, 0, 0; 
                        0, 1, 0, 0, 0, 0, 0; 
                        0, 0, 1, 0, 0, 0, 0; 
                        0, 0, 0, 1, 0, 0, 0; 
                        0, 0, 0, 0, 1, 0, 0; 
                        0, 0, 0, 0, 0, 1, 0; 
                        0, 0, 0, 0, 0, 0, 1];

                A_d = I + A*self.Ts + A^2*self.Ts^2; 
                % update P with discrete time model
                self.P = A_d*self.P*A_d' + self.Ts^2*self.Q;  %-------IDK MAN 
            end
        end
        function self = measurement_update(self, state, measurement)
            % always update based on wind triangle pseudu measurement
%            h = self.h_pseudo(self.xhat, state);
 %           C = self.jacobian(@self.h_pseudo, self.xhat, state);
  %          y = [measurement.accel_x; measurement.accel_y; measurement.accel_x]; 
   %         for i=1:2 %was 2
    %            Ci = 
     %           L = 
      %          self.P = 
       %         self.xhat = 
        %    end

            %# only update GPS when one of the signals changes
            if (measurement.gps_n ~= self.gps_n_old)...
                || (measurement.gps_e ~= self.gps_e_old)...
                || (measurement.gps_Vg ~= self.gps_Vg_old)...
                || (measurement.gps_course ~= self.gps_course_old)

                h = self.h_gps(self.xhat, state);
                C = self.jacobian(@self.h_gps, self.xhat, state);
                y = [measurement.gps_n; measurement.gps_e; measurement.gps_Vg; measurement.gps_course ];
                for i=1:4
                    Ci = C(i,: );
                    L = (self.P*Ci')/(self.R(i)+Ci*self.P*Ci'); 
                    I = [ %Identity matrix
                            1, 0, 0, 0, 0, 0, 0; 
                            0, 1, 0, 0, 0, 0, 0; 
                            0, 0, 1, 0, 0, 0, 0; 
                            0, 0, 0, 1, 0, 0, 0; 
                            0, 0, 0, 0, 1, 0, 0; 
                            0, 0, 0, 0, 0, 1, 0; 
                            0, 0, 0, 0, 0, 0, 1];
                    self.P = (I-L*Ci)*self.P*(I-L*Ci)' + L*self.R(i)*L';
                    self.xhat = self.xhat + L*(y(i)-h(i)); 
                end

                % update stored GPS signals
                self.gps_n_old = measurement.gps_n;
                self.gps_e_old = measurement.gps_e;
                self.gps_Vg_old = measurement.gps_Vg;
                self.gps_course_old = measurement.gps_course;
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