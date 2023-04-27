% path follower for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/12/2019 - RWB
classdef path_follower < handle
   %--------------------------------
    properties
        chi_infty
        k_path
        k_orbit
        gravity
        commands
        %phi_feedforward %--------------trying something out for chapoter 10 
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_follower
            self.chi_infty = pi/4; %0.5;
            self.k_path = 0.02; %0.5;
            self.k_orbit = 4.3; %0.5;
            self.gravity = 9.8;
            addpath('../message_types'); 
            self.commands = msg_autopilot();
        end
        %------methods-----------
        function autopilot_commands = update(self, path, state)
            if isequal(path.flag, 'line')
                self.follow_straight_line(path, state);
            else % path.flag == 'orbit'
                self.follow_orbit(path, state);
            end
            % return the estimated state
            autopilot_commands = self.commands;
        end
        %---------------------------
        function self = follow_straight_line(self, path, state)
            self.commands.airspeed_command = path.airspeed;
            chi_q = atan2(path.line_direction(2), path.line_direction(1));
            chi = state.chi;
            chi_c = chi_q - self.chi_infty*(pi/180)*sign(sin(chi_q-chi));
            self.commands.course_command = self.wrap(chi_c, chi);
            self.commands.altitude_command = path.altitude;
            self.commands.phi_feedforward = 0;
        end
        %---------------------------
        function self = follow_orbit(self, path, state)
            self.commands.airspeed_command = path.airspeed;
            %q_i = path.center;  % <-- Replace 'origin' with 'center'
            c = path.orbit_center;
            rho = path.orbit_radius;
            lambda = path.orbit_direction;
            d = norm(state.p - c);
            phi = atan2(state.p-c(2), state.p-c(1)); %---------this migght casue issues later... 
            if lambda == 1
                chi_q = phi + pi/2;
            else
                chi_q = phi - pi/2;
            end
            chi = state.chi;
            chi_c = chi_q + self.k_orbit*(d-rho)*sign(sin(chi_q-chi));
            self.commands.course_command = self.wrap(chi_c, chi);
            self.commands.altitude_command = path.altitude + rho*sin((pi/2) * ((d-rho)/rho));
            self.commands.phi_feedforward = atan2(self.gravity*self.commands.airspeed_command^2, self.k_path*self.gravity);
        end
        %---------------------------
        function chi_c = wrap(self, chi_c, chi)
            while chi_c-chi > pi
                chi_c = chi_c - 2*pi;
            end
            while chi_c-chi < -pi
                chi_c = chi_c + 2*pi;
            end
        end
    end
end