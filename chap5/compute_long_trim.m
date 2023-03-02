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
            
            
            %TODO figure out how to set the longitudinal conditions
            %specified by the constructor function above.
            %Then apply the delta inputs to the vehicle model at that
            %condition and return the longitudinal forces.
            %Then take the norm of those forces and return them in
            %norm_long_forces.  Remember you have the mav dynamics in the
            %self.mav object and the mav parameters in the self.MAV object
            norm_long_forces = 0;
        end
    end
end