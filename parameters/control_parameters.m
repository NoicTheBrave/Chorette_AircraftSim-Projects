% control_parameters
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
%         3/01/2023 - LRH Modified heavily
%addpath('../chap5')
%load transfer_function_coef

% AP stands for autopilot
AP.gravity = 0;
AP.sigma = 0;
AP.Va0 = 0;

%----------roll rate loop-------------
AP.p_kp = 0;
AP.p_ki = 0;
AP.p_kd = 0;

%----------roll loop-------------
AP.phi_kp = 0;
AP.phi_ki = 0;
AP.phi_kd = 0;

%----------course loop-------------
AP.chi_kp = 0;
AP.chi_ki = 0;
AP.chi_kd = 0;

%----------sideslip loop-------------
AP.beta_kp = 0;
AP.beta_ki = 0;
AP.beta_kd = 0;

%----------pitch rate loop-------------
AP.q_kp = -4.6;%-3.5;
AP.q_ki = .0001% -.1;
AP.q_kd = 0;

%----------flight path angle loop-------------
AP.gamma_kp = 1;
AP.gamma_ki = 0;
AP.gamma_kd = 0;

%----------altitude loop-------------
AP.h_kp = 0;
AP.h_ki = 0;
AP.h_kd = 0;

%---------airspeed loop---------------
AP.Va_kp = 1;
AP.Va_ki = .01;
AP.Va_kd = -0.01;
