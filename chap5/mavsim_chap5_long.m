%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 5 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         2/5/2019 - RWB
%         2/27/2023 - LRH Heavily Modified for longitudinal only
        %3/1/2023 - NIC Doesnt know what the hell he is doing
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

% compute longitudinal trim
addpath('../chap5');

Va = 20;  %TODO set your desired Va speed
gamma = deg2rad(12);  %TODO set your desired gamma speed in radians

%Setup trim class with desired trim conditions
trim = compute_long_trim(Va, gamma, mav, MAV);

%Guess input conditions
delta_e_0 = 0;  %TODO guess your trimmed delta_e
delta_t_0 = 0;   %TODO guess your trimmed delta_t
alpha_0 = 0; %TODO guess your trimmed alpha

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

out = trim.compute_long_forces(x)
mav.set_longitudinal(Va, gamma, x(1));
delta = [x(2), x(3), 0, 0];

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time

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
    mav_view.set_view(90, 0)
    if VIDEO==1
        video.update(sim_time);  
    end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1
    video.close(); 
end

