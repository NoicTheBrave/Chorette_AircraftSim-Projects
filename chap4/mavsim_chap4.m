%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 4 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         12/27/2018 - RWB
%         1/18/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
restoredefaultpath
addpath('../chap2'); 
addpath('../chap3'); 
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters

% initialize the mav viewer
mav_view = mav_viewer();  
data_view = data_viewer();

% initialize the video writer
VIDEO = 1;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap4_video.avi', SIM.ts_video); end

% initialize elements of the architecture
addpath('../chap4'); wind = wind_simulation(SIM.ts_simulation);
mav = mav_dynamics(SIM.ts_simulation, MAV);

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------set control surfaces-------------
    delta_e = -0.2;
    delta_t = 0.5;
    delta_a = -0.01;  
    delta_r = 0;
    delta = [delta_e; delta_t; delta_a; delta_r];

    %-------physical system-------------
    current_wind = wind.update();
    mav.update_state(delta, current_wind, MAV);
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);  % plot body of MAV
    data_view.update(mav.true_state,... % true states
                     mav.true_state,... % estimated states
                     mav.true_state,... % commmanded states
                     SIM.ts_simulation); 
    if VIDEO, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO, video.close(); end

