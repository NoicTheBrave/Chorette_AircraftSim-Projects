classdef compute_long_trim < handle
    %contains the setup to trim the longitudinal axis

    properties
        Va
        gamma
        mav
        MAV
    end

    methods
        function self = compute_long_trim(Va, gamma, mav, MAV)
            %Setup trim state and mav properties
            self.Va = Va;
            self.gamma = gamma;
            self.mav = mav;
            self.MAV = MAV;
        end

        function norm_long_forces = compute_long_forces(self, x)
            %compute longitudinal forces with an input vector
            % x = [alpha, delta_e, delta_t] and output the weighted root sum 
            % squared of fx, fz, and m
            alpha = x(1); 
            delta_e = x(2); 
            delta_t = x(3); 

            
            self.mav.set_longitudinal(self.Va, self.gamma, alpha );
            delta = [x(2), x(3), 0, 0];
            f_and_m = self.mav.forces_moments(delta, self.MAV);
            
            f_x = f_and_m(1);
            f_z = f_and_m(3);
            m = f_and_m(5);

            norm_long_forces = norm([f_x, f_z, m]);
            %f_out(1) = f_and_m(1);
            %f_out(2) = f_and_m(3);
            %f_out(3) = f_and_m(5);
            %norm_long_forces = norm(f_out);
            

            %TODO figure out how to set the longitudinal conditions
            %specified by the constructor function above.
            %Then apply the delta inputs to the vehicle model at that
            %condition and return the longitudinal forces.
            %Then take the norm of those forces and return them in
            %norm_long_forces.  Remember you have the mav dynamics in the
            %self.mav object and the mav parameters in the self.MAV object
            %norm_long_forces = 0;
        end
    end
end