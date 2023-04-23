% pid_control
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
classdef pid_control < handle
   %--------------------------------
    properties
        kp
        ki
        kd
        Ts
        low_limit
        up_limit
        integrator
        last_error
        error_delay_1
        error_dot_delay_1
        a1
        a2
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = pid_control(kp, ki, kd, Ts, low_limit, up_limit)
            self.kp = kp;
            self.ki = ki;
            self.kd = kd;
            self.Ts = Ts;
            self.low_limit = low_limit;
            self.up_limit = up_limit;
            self.integrator = 0;
%             self.error_delay_1 = 0;
%             self.error_dot_delay_1 = 0;
%             self.a1 = (2*sigma-Ts)/(2*sigma+Ts);
%             self.a2 = 2/(2*sigma+Ts);
            self.last_error = -9999;
        end
        %----------------------------
        function reset_integrator(self, reset)
            if(self.ki ~= 0)
                self.integrator = reset / self.ki;
            end
        end
        %----------------------------
        function u_sat = update(self, y_ref, y)
            error = y_ref - y;
            p = error * self.kp;
            i = self.integrator * self.ki;
            if (self.last_error == -9999)
                self.last_error = error;
            end
            d = (error - self.last_error) * self.kd/self.Ts;
            u = p + i + d;
            u_sat = self.saturate(u);
            self.integrator = self.integrator + error * self.Ts;
        end
        %----------------------------
        function u_sat = update_with_rate(self, y_ref, y, ydot)
            error = yref - y;
            p = error * self.kp;
            i = self.integrator * self.ki;
            self.last_error = error;
            d = ydot * self.kd/self.Ts;
            u = p + i + d;
            u_sat = self.saturate(u);
            self.integrator = self.integrator + error * self.Ts;
        end
        %----------------------------
        function out = saturate(self, in)
            % saturate u at +- self.limit
            if in >= self.up_limit
                out = self.up_limit;
            elseif in <= self.low_limit
                out = self.low_limit;
            else
                out = in;
            end
        end
    end
end