% autopilot block for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
classdef autopilot < handle
   %--------------------------------
    properties
        ts_control
        rollrate_aileron
        rollangle_rollrate
        courseangle_rollangle
        sideslip_rudder
        pitchrate_elevator
        gamma_pitchrate
        alt_gamma
        Va_throttle
        commanded_state
        vertical_ap_state
        horizontal_ap_state
        trim_delta
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = autopilot(ts_control)
             % load AP: control gains/parameters
            run('../parameters/control_parameters') 
            addpath('../chap6')
            self.ts_control = ts_control;
            self.rollrate_aileron = pid_control(AP.p_kp, AP.p_ki, AP.p_kd, self.ts_control, -1, 1);
            self.rollangle_rollrate = pid_control(AP.phi_kp, AP.phi_ki, AP.phi_kd, self.ts_control, deg2rad(-20), deg2rad(20));
            self.courseangle_rollangle = pid_control(AP.chi_kp, AP.chi_ki, AP.chi_kd, self.ts_control, deg2rad(-40), deg2rad(40));
            self.sideslip_rudder = pid_control(AP.beta_kp, AP.beta_ki, AP.beta_kd, self.ts_control, -1, 1);
            self.pitchrate_elevator = pid_control(AP.q_kp, AP.q_ki, AP.q_kd, self.ts_control, -1, 1);
            self.gamma_pitchrate = pid_control(AP.gamma_kp, AP.gamma_ki, AP.gamma_kd, self.ts_control, deg2rad(-20), deg2rad(20));
            self.alt_gamma = pid_control(AP.h_kp, AP.h_ki, AP.h_kd, self.ts_control, deg2rad(-10), deg2rad(20));
            self.Va_throttle = pid_control(AP.Va_kp, AP.Va_ki, AP.Va_kd, self.ts_control, 0, 1);
            addpath('../message_types'); 
            self.commanded_state = msg_state();
            self.vertical_ap_state = 'none';
            self.horizontal_ap_state = 'none';
            self.trim_delta = [0, 0, 0, 0];
        end
        %------methods-----------
        function set_trim_delta(self, delta)
            self.trim_delta = delta;
            self.pitchrate_elevator.reset_integrator(delta(1));
            self.Va_throttle.reset_integrator(delta(2));
        end
        function set_ap_modes(self, horiz_state, vert_state)
            if strcmp(horiz_state, 'p') || strcmp(horiz_state, 'phi') || strcmp(horiz_state, 'chi')
                self.horizontal_ap_state = horiz_state;
            else
                print('No horizontal state %s is defined', horiz_state)
            end
            if strcmp(vert_state, 'q') || strcmp(vert_state, 'gamma') || strcmp(vert_state, 'alt')
                self.vertical_ap_state = vert_state;
            else
                print('No vertical state %s is defined', vert_state)
            end
        end
        function [delta, commanded_state] = update(self, cmd, state)
            
            % lateral autopilot
            % longitudinal autopilot
            if(strcmp(self.horizontal_ap_state,'p'))
                p_c = cmd.p_command;
                self.commanded_state.p = cmd.p_command;
                self.commanded_state.phi = 0;
                self.commanded_state.chi = 0;
                delta_a = self.rollrate_aileron.update(p_c, state.p);
          
            elseif(strcmp(self.horizontal_ap_state,'phi'))
                phi_c = cmd.phi_command;
                self.commanded_state.phi = cmd.phi_command;
                self.commanded_state.chi = 0;
                p_c = self.rollangle_rollrate.update(phi_c, state.phi);
                self.commanded_state.p = p_c;
                delta_a = self.rollrate_aileron.update(p_c, state.p);
                
            elseif(strcmp(self.horizontal_ap_state,'chi'))
                chi_c = cmd.course_command;
                self.commanded_state.chi = cmd.course_command;
                phi_c = self.courseangle_rollangle.update(chi_c, state.chi);
                self.commanded_state.phi = phi_c;
                p_c = self.rollangle_rollrate.update(phi_c, state.phi);
                self.commanded_state.p = p_c;
                delta_a = self.rollrate_aileron.update(p_c, state.p);
                
            else
                ME = MException('Incorrect State', ...
                    'State %s not included in the possible lateral autopilot modes',self.vertical_ap_state);
                throw(ME)
            end

            self.commanded_state.beta = 0;
            delta_r = self.sideslip_rudder.update(0, state.beta);

            % longitudinal autopilot
            if(strcmp(self.vertical_ap_state,'q'))
                q_c = cmd.q_command;
                self.commanded_state.q = cmd.q_command;
                self.commanded_state.gamma = 0;
                self.commanded_state.h = 0;
                delta_e = self.pitchrate_elevator.update(q_c, state.q);
                %delta_e_add = self.pitchrate_elevator.update_with_no_rate(p_c, state.p);
                %delta_e = self.saturate(self.trim_delta(1) + delta_e_add, -1, 1);
          
            elseif(strcmp(self.vertical_ap_state,'gamma'))
                gamma_c = cmd.gamma_command;
                self.commanded_state.gamma = cmd.gamma_command;
                self.commanded_state.h = 0;
                q_c = self.pitchrate_elevator.update(gamma_c, state.gamma);
                self.commanded_state.q = q_c;
                delta_e = self.pitchrate_elevator.update(q_c, state.q);
                
            elseif(strcmp(self.vertical_ap_state,'alt'))
                h_c = cmd.altitude_command;
                self.commanded_state.h = cmd.altitude_command;
                gamma_c = self.alt_gamma.update(h_c, state.h);
                self.commanded_state.gamma = gamma_c;
                q_c = self.gamma_pitchrate.update(gamma_c, state.gamma);
                self.commanded_state.q = q_c;
                delta_e = self.pitchrate_elevator.update(q_c, state.q);
                
            else
                ME = MException('Incorrect State', ...
                    'State %s not included in the possible longitudinal autopilot modes',self.vertical_ap_state);
                throw(ME)
            end

            Va_c = cmd.airspeed_command;
            self.commanded_state.Va = Va_c;
            delta_t = self.Va_throttle.update(Va_c, state.Va);
             

            % construct output and commanded states
            delta = [delta_e; delta_t; delta_a; delta_r];
            
%             self.commanded_state.Va = cmd.airspeed_command;
%             self.commanded_state.phi = phi_c;
%             self.commanded_state.theta = theta_c;
%             self.commanded_state.chi = cmd.course_command;
        	commanded_state = self.commanded_state;
        end
        %---------------------------
        function output = saturate(self, input, low_limit, up_limit)
            if input <= low_limit
                output = low_limit;
            elseif input >= up_limit
                output = up_limit;
            else
                output = input;
            end
        end
    end
end