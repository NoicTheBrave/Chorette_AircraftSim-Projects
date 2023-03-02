clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 2 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         12/15/2018 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run('../parameters/simulation_parameters.m')  % load SIM: simulation parameters

% initialize messages
addpath('../message_types'); state = msg_state();  

% initialize the mav viewer
addpath('../chap2'); mav_view = MAV_viewer();
%addpath('C:/Users/Mhubb/OneDrive/Documents/MATLAB/NewAC Sim/chap2'); mav_view = spacecraft_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap2_video.avi', SIM.ts_video); end

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------vary states to check viewer-------------
    % animation changed to right pull up, then stall to freefall
    if sim_time < SIM.end_time/6
        state.pn = state.pn + 0.6*SIM.ts_simulation;
    elseif sim_time < 2*SIM.end_time/6
        state.pn = state.pn + 0.4*SIM.ts_simulation;
        state.pe = state.pe + 0.6*SIM.ts_simulation;
        state.theta = state.theta + 0.1*SIM.ts_simulation;
        state.h = state.h + 0.6*SIM.ts_simulation;
        state.psi = state.psi + 0.1*SIM.ts_simulation;
    else 
        state.pn = state.pn + 0.02*SIM.ts_simulation;
        state.theta = state.theta - 0.1*SIM.ts_simulation;
        state.h = state.h - 0.6*SIM.ts_simulation;
        state.psi = state.psi - 0.1*SIM.ts_simulation;
    end

    %-------update viewer-------------
    mav_view.update(state);
    if VIDEO, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO, video.close(); end
