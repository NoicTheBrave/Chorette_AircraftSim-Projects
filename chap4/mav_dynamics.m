% mav dynamics - implement rigid body dynamics for mav
%
% mavMatSim 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         1/18/2019 - RWB
%         2/15/2023 - LRH
classdef mav_dynamics < handle
   %--------------------------------
    properties
        ts_simulation
        state
        Va
        alpha
        beta
        wind
        true_state
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts; % time step between function calls
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            self.Va = 0;  %TODO
            self.alpha = 0;  %TODO
            self.beta = 0;  %TODO
            self.wind = 0;  %TODO
            addpath('../message_types'); self.true_state = msg_state();
        end
        %---------------------------
        function self=update_state(self, delta, wind, MAV)
            %
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV.
            % 
            
            % get forces and moments acting on rigid body
            forces_moments = self.forces_moments(delta, MAV);
            
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % normalize the quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            
            % update the airspeed, angle of attack, and side slip angles
            self.update_velocity_data(wind);
            
            % update the message class for the true state
            self.update_true_state();
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
            pn    = state(1);
            pe    = state(2);
            pe    = state(3);
            u     = state(4);
            v     = state(5);
            w     = state(6);
            e0    = state(7);
            e1    = state(8);
            e2    = state(9);
            e3    = state(10);
            p     = state(11);
            q     = state(12);
            r     = state(13);
            fx    = forces_moments(1);
            fy    = forces_moments(2);
            fz    = forces_moments(3);
            ell   = forces_moments(4);
            m     = forces_moments(5);
            n     = forces_moments(6);
        
            % position kinematics
            pn_dot = 0;  %TODO
            pe_dot = 0;  %TODO
            pd_dot = 0;  %TODO

            % position dynamics
            u_dot = 0;  %TODO
            v_dot = 0;  %TODO
            w_dot = 0;  %TODO
            
            % rotational kinematics
            e0_dot = 0;  %TODO
            e1_dot = 0;  %TODO
            e2_dot = 0;  %TODO
            e3_dot = 0;  %TODO

            % rotational dynamics
            p_dot = 0;  %TODO
            q_dot = 0;  %TODO
            r_dot = 0;  %TODO
                
            % collect all the derivaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
            self.wind = wind;
            self.Va = 0;  %TODO
            self.alpha = 0;  %TODO
            self.beta = 0;  %TODO
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
            f_x_g = 0;  %TODO
            f_y_g = 0;  %TODO
            f_z_g = 0;  %TODO
           
            f_lift = 0;  %TODO
            f_drag = 0;  %TODO
            f_x_aero = 0;  %TODO
            f_z_aero = 0;  %TODO
            
            f_y_aero = 0;  %TODO

            f_x_thrust = 0;  %TODO
 
            f_x = f_x_g + f_x_aero + f_x_thrust;
            f_y = f_y_g + f_y_aero;
            f_z = f_z_g + f_z_aero;

            ell = 0;  %TODO% moment about the x axis
            m = 0;  %TODO% moment about the y axis
            n = 0;  %TODO% moment about the z axis
            
            Force = [f_x, f_y, f_z];
            Torque = [ell, m, n];
            % output total force and torque
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_true_state(self)
            [phi, theta, psi] = Quaternion2Euler(self.state(7), self.state(8), self.state(9), self.state(10));
            self.true_state.pn = self.state(1);  % pn
            self.true_state.pe = self.state(2);  % pd
            self.true_state.h = -self.state(3);  % h
            self.true_state.phi = phi; % phi
            self.true_state.theta = theta; % theta
            self.true_state.psi = psi; % psi
            self.true_state.p = self.state(11); % p
            self.true_state.q = self.state(12); % q
            self.true_state.r = self.state(13); % r
            self.true_state.Va = self.Va;
            self.true_state.alpha = self.alpha;
            self.true_state.beta = self.beta;
            self.true_state.Vg = 0;  %TODO
            self.true_state.chi = 0;  %TODO
            self.true_state.gamma = 0;  %TODO
            self.true_state.wn = self.wind(1);
            self.true_state.we = self.wind(2);
        end
    end
end