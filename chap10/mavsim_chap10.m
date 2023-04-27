%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavsim_matlab 
%     - Chapter 10 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         3/12/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
restoredefaultpath


clear all, path(pathdef)  % clear all variables and reset path 
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters
run('../parameters/control_parameters')  % load CTRL: control parameters
run('../parameters/sensor_parameters')  % load SENSOR: sensor parameters

% initialize the viewer
addpath('../chap10'); path_view = path_viewer();
addpath('../chap3'); data_view = data_viewer(); % use for debugging

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap10_video.avi', SIM.ts_video); end

% initialize elements of the architecture
%addpath('../chap7'); 
mav = mav_dynamics(SIM.ts_simulation, MAV);
addpath('../chap4');  wind = wind_simulation(SIM.ts_simulation);
addpath('../chap6');  ctrl = autopilot(SIM.ts_simulation);
addpath('../chap8');  obsv = observer(SIM.ts_simulation);
addpath('../chap10'); path_follow = path_follower();

% path definition
addpath('../message_types'); path = msg_path();%addpath('../message_types'); path = msg_path();
addpath('../tools'); %--------------------------I CHOSE TO ADD THIS IN HOPES THINGS WILL STOP BREAKING AS MUCH 

%path.flag = 'line';

path.flag = 'orbit'; %--------------this was origionally enabled 

%path.type = 'orbit'; %--testing out something new... 
if isequal(path.flag, 'line')
    path.line_origin = [0.0; 0.0; -100.0];
    path.line_direction = [0.5; 1.0; 0.0];
    path.line_direction = path.line_direction / norm(path.line_direction);
else  % path.flag == 'orbit'
    path.orbit_center = [0.0; 0.0; -100.0];  
    path.orbit_radius = 200.0;  
    path.orbit_direction = 'CW';  
end

% initialize the simulation time
sim_time = SIM.start_time;







%-----------------aDDITIONAL ADDED THINGS TO TRY AND COMPENSATE FOR THE bs
%OF THE CODE ERROR
% arguments to signals are amplitude, frequency, start_time, dc_offset
Va_command = signals(3, 0.01, 2, 25);
h_command = signals(10, 0.02, 0, 100);
chi_command = signals(45*pi/180, 0.015, 5, 180*pi/180);

% initialize the simulation time
%sim_time = SIM.start_time;

%est_viewer = est_state_viewer();



% compute longitudinal trim
addpath('../chap5');
Va = 20;
gamma = deg2rad(0);
%Setup trim class with desired trim conditions
trim = compute_long_trim(Va, gamma, mav, MAV);

%Guess input conditions
delta_e_0 = -0.2;  %-.2
delta_t_0 = .7;   %.5
alpha_0 = deg2rad(2.0);

x = [alpha_0, delta_e_0, delta_t_0];

%trim the airplane in the longitudinal axis
A = [];
b = [];
Aeq = [];
beq = [];
lb = [deg2rad(-5), -1, 0];
ub = [deg2rad(20), 1, 1];
x0 = [alpha_0, delta_e_0, delta_t_0];
x = fmincon(@trim.compute_long_forces, x0, A, b, Aeq, beq, lb, ub);

out = trim.compute_long_forces(x);
mav.set_longitudinal(Va, gamma, x(1));
delta = [x(2), x(3), 0, 0];


ctrl.set_trim_delta(delta);
ctrl.set_ap_modes('chi', 'alt');
p_command = signals(deg2rad(5), pi/2, 1, 0);
q_command = signals(deg2rad(10), pi/2, 1, 0);
phi_command = signals(deg2rad(5), pi/5, 1, 0);
gamma_command = signals(deg2rad(5), pi/5, 1, 0);
course_command = signals(deg2rad(50), pi/10, 1, 0);
altitude_command = signals(10, pi/10, 1, 100); %altitude_command = signals(10, pi/10, 1, 100);
Va_command = 20;

 sim_time = SIM.start_time; 







% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    
    %-------observer-------------
    mav.update_sensors(MAV, SENSOR); % get sensor measurement <----------- it appears that it is here where shit hits the fan 
    measurements = mav.sensors;
    estimated_state = obsv.update(measurements, MAV);

    %-------path follower-----
    autopilot_commands = path_follow.update(path, estimated_state);

    %-------controller-------------
    [delta, commanded_state] = ctrl.update(autopilot_commands, estimated_state);
    % use true states for debugging
    %[delta, commanded_state] = ctrl.update(autopilot_commands, mav.true_state);


    delta = [delta(1), delta(2), delta(3), 0]; %FORCED OVERRIDE


    %-------physical system-------------
    %current_wind = wind.update();
    %mav.update(delta, current_wind, MAV); %Have not used wind yet
    current_wind = wind.update();
    mav.update_state(delta, current_wind, MAV, "all");


    %-------update viewer-------------
    path_view.update(path, mav.true_state);
    % use for debugging
%     data_view.update(mav.true_state,...
%                      estimated_state,...
%                      commanded_state,...
%                      SIM.ts_simulation); 
    if VIDEO==1, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1, video.close(); end

