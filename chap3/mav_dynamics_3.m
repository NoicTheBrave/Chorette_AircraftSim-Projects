% mav dynamics - implement rigid body dynamics for mav
%
% mavsim
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         12/19/2018 - RWB
classdef mav_dynamics_3 < handle
   %--------------------------------
    properties
        ts_simulation
        state
        true_state
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics_3(Ts, MAV)
            self.ts_simulation = Ts; % time step between function calls
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            addpath('../message_types'); self.true_state = msg_state();
        end
        %---------------------------
        function self=update_state(self, forces_moments, MAV)
            %
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV.
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % normalize the quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            self.update_true_state();
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
            pn    = state(1);
            pe    = state(2);
            pd    = state(3); % needs to be corrected to pd?
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
            
            Reib = [e0^2+e1^2-e2^2-e3^2, 2*(e1*e2-e0*e3), 2*(e1*e3+e0*e2);
                    2*(e1*e2+e0*e3), e0^2-e1^2+e2^2-e3^2, 2*(e2*e3-e0*e1);
                    2*(e1*e3-e0*e2), 2*(e2*e3+e0*e1), e0^2-e1^2-e2^2+e3^2]*[u; v; w];
            pn_dot = Reib(1);
            pe_dot = Reib(2);
            pd_dot = Reib(3);

            % position dynamics
            u_dot = r*v-q*w+(1/MAV.mass)*fx;
            v_dot = p*q-r*u+(1/MAV.mass)*fy;
            w_dot = q*u-p*v+(1/MAV.mass)*fz;
            
            % rotational kinematics
            e = .5*[0, -p, -q, -r;
                    p, 0, r, -q;
                    q, -r, 0, p;
                    r, q, -p, 0]*[e0; e1; e2; e3];
                    
            e0_dot = e(1);
            e1_dot = e(2);
            e2_dot = e(3);
            e3_dot = e(4);
            
            % rotational dynamics
            g1 = MAV.Gamma1;
            g2 = MAV.Gamma2;
            g3 = MAV.Gamma3;
            g4 = MAV.Gamma4;
            g5 = MAV.Gamma5;
            g6 = MAV.Gamma6;
            g7 = MAV.Gamma7;
            g8 = MAV.Gamma8;
            p_dot = (g1*p*q-g2*q*r)+(g3*ell+g4*n);
            q_dot = (g5*p*r-g6*(p^2-r^2))+((1/MAV.Jy)*m);
            r_dot = (g7*p*q-g1*q*r)+(g4*ell+g8*n);
        
            
            % collect all the derivaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_true_state(self)
            e = self.state(7:10);
            e0 = e(1);
            e1 = e(2);
            e2 = e(3);
            e3 = e(4);
            angles = self.Quaternion2Euler(e0, e1, e2, e3);
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            self.true_state.pn = self.state(1);  % pn
            self.true_state.pe = self.state(2);  % pe
            self.true_state.h = -self.state(3);  % h
            self.true_state.phi = phi; % phi
            self.true_state.theta = theta; % theta
            self.true_state.psi = psi; % psi
            self.true_state.p = self.state(11); % p
            self.true_state.q = self.state(12); % q
            self.true_state.r = self.state(13); % r
        end
        
        function angles = Quaternion2Euler(self, e0, e1, e2, e3)
           
            % roll (x-axis rotation)
            sinr_cosp = 2 * (e0 * e1 + e2 * e3);
            cosr_cosp = 1 - 2 * (e1 * e1 + e2 * e2);
            phi = atan2(sinr_cosp, cosr_cosp);

            % pitch (y-axis rotation)
            sinp = sqrt(1 + 2 * (e0 * e2 - e1 * e3));
            cosp = sqrt(1 - 2 * (e0 * e2 - e1 * e3));
            theta = 2 * atan2(sinp, cosp) - pi / 2;

            % yaw (z-axis rotation)
            siny_cosp = 2 * (e0 * e3 + e1 * e2);
            cosy_cosp = 1 - 2 * (e2 * e2 + e3 * e3);
            psi = atan2(siny_cosp, cosy_cosp);
            angles = [phi, theta, psi];

        end
    end
end