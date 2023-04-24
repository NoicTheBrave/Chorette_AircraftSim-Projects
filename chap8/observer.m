% observer for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef observer < handle
   %--------------------------------
    properties
        estimated_state
        lpf_gyro_x
        lpf_gyro_y
        lpf_gyro_z
        lpf_accel_x
        lpf_accel_y
        lpf_accel_z
        lpf_static
        lpf_diff
        attitude_ekf
        position_ekf
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = observer(ts_control)
            addpath('../message_types'); 
            self.estimated_state = msg_state();
            addpath('../chap8')
            self.lpf_gyro_x = alpha_filter(0.5);
            self.lpf_gyro_y = alpha_filter(0.5);
            self.lpf_gyro_z = alpha_filter(0.5);
            self.lpf_accel_x = alpha_filter(0.5);
            self.lpf_accel_y = alpha_filter(0.5);
            self.lpf_accel_z = alpha_filter(0.5); %some of these were like 0.99, the lower ones, not sure how many... 
            self.lpf_static = alpha_filter(0.8); %0.5
            self.lpf_diff = alpha_filter(0.9);% 0.9 %was -.5
            self.attitude_ekf = ekf_attitude();% 
            self.position_ekf = ekf_position();
%             % load AP: control gains/parameters
%             run('../parameters/control_parameters') 

% 
        end
        %------methods-----------
        function estimated_state = update(self, measurements, MAV)
            % estimates for p, q, r are low pass filter of gyro minus bias estimate
            self.estimated_state.p = self.lpf_gyro_x.update(measurements.gyro_x) ;%- SENSOR.gyro_x_bias;
            self.estimated_state.q = self.lpf_gyro_y.update(measurements.gyro_y) ;%- SENSOR.gyro_y_bias;
            self.estimated_state.r = self.lpf_gyro_z.update(measurements.gyro_z) ;%- SENSOR.gyro_z_bias;

            % invert sensor model to get altitude and airspeed
            self.estimated_state.h = self.lpf_static.update(measurements.static_pressure) / (measurements.rho * -9.81);
            self.estimated_state.Va = sqrt((2/measurements.rho)*self.lpf_diff.update(measurements.diff_pressure)); 

            self.estimated_state.theta = asin(self.lpf_accel_x.update(measurements.accel_x)/9.81); 
            self.estimated_state.phi = atan(self.lpf_accel_y.update(measurements.accel_y)/self.lpf_accel_z.update(measurements.accel_z)); 



            % estimate phi and theta with simple ekf
            self.estimated_state = self.attitude_ekf.update(self.estimated_state, measurements);

            % estimate pn, pe, Vg, chi, wn, we, psi
            self.estimated_state = self.position_ekf.update(self.estimated_state, measurements);

            % not estimating these
            self.estimated_state.alpha = self.estimated_state.theta;
            self.estimated_state.beta = 0.0;
            self.estimated_state.bx = 0.0;
            self.estimated_state.by = 0.0;
            self.estimated_state.bz = 0.0;
            
            % return the estimated state
            estimated_state = self.estimated_state;
        end
    end
end