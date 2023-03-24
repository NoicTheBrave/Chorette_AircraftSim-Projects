%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 5 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         2/5/2019 - RWB
%         2/27/2023 - LRH Heavily Modified for longitudinal only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
restoredefaultpath
addpath('../chap2'); 
addpath('../chap3'); 
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters

% initialize the mav viewer
mav_view = mav_viewer();  
data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1
    video=video_writer('chap5_video.avi', SIM.ts_video);
end

% initialize elements of the architecture
addpath('../chap4'); 
wind = [0,0,0,0,0,0];
mav = mav_dynamics(SIM.ts_simulation, MAV);
addpath('../chap6'); 
ctrl = autopilot(SIM.ts_simulation);
ctrl_viewer = control_viewer();

addpath('../message_types'); commands = msg_autopilot();
addpath('../tools');

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

% arguments to signals are amplitude, frequency, start_time, dc_offset
ctrl.set_trim_delta(delta);
ctrl.set_ap_modes('p', 'q');
p_command = signals(deg2rad(5), pi, 1, 0);
q_command = signals(deg2rad(10), pi/2, 1, 0);
gamma_command = signals(deg2rad(5), pi, 1, 0);
Va_command = 20;

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time

    %-------controller-------------
    estimated_state = mav.true_state;  % uses true states in the control
    commands.p_command = p_command.square(sim_time);
    commands.q_command = q_command.square(sim_time);
    %commands.gamma_command = gamma_command.square(sim_time);
    commands.airspeed_command = Va_command;
    [delta, commanded_state] = ctrl.update(commands, estimated_state);

    %-------physical system-------------
    %current_wind = wind.update();
    current_wind = zeros(6,1);
    mav.update_state(delta, current_wind, MAV, 'long');
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);  % plot body of MAV
    data_view.update(mav.true_state,...  % true states
                     mav.true_state,...  % estimated states
                     mav.true_state,...  % commmanded states
                     SIM.ts_simulation);
    mav_view.set_view(90, 0);
    ctrl_viewer.update(mav.true_state, commanded_state, delta, SIM.ts_simulation);
    if VIDEO==1
        video.update(sim_time);  
    end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1
    video.close(); 
end

