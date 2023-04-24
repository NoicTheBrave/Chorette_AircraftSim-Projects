% alpha_filter
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef alpha_filter < handle
   %--------------------------------
    properties
        alpha
        y
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = alpha_filter(alpha)
            self.alpha = alpha;
            self.y = 0;
        end
        %------methods-----------
        function y = update(self, u)
            
            if(self.y == 0) %compensates for matlab's goofyness
                self.y = u; 
                y = self.y; 
            else
                self.y = self.alpha*self.y + (1-self.alpha)*u;% 0.5 ; %this is a guess 
                y = self.y;
            end %end of IF-ELSE 
        end
    end
end