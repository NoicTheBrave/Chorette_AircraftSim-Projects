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
            self.mav.set_longitudinal(self.Va, self.gamma, x(1));
            delta = [x(2), x(3), 0, 0];
            f_and_m = self.mav.forces_moments(delta, self.MAV);
            f_out(1) = f_and_m(1);
            f_out(2) = f_and_m(3);
            f_out(3) = f_and_m(5);
            norm_long_forces = norm(f_out);
        end
    end
end