% msg_autopilot
%   - message type for commands sent to the autopilot
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         2/12/2019 - RWB
%         3/1/2023 - LRH Added command messages
classdef msg_autopilot
   %--------------------------------
    properties
        %Lateral commands beta_command = 0 implicitly
        p_command
        phi_command
        course_command
        %Longitudinal
        q_command
        gamma_command
        altitude_command
        airspeed_command
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_autopilot()
            
            self.p_command = 0;
            self.phi_command = 0;
            self.course_command = 0;
            
            self.q_command = 0;
            self.gamma_command = 0;
            self.altitude_command = 0;
            self.airspeed_command = 0;
        end
    end
end