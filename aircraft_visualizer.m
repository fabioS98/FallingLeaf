%% Example script to visualize the aircraft simulation data
% Add the path of the aircraft_3d_animation function
%addpath('../src/');
% path of the *.mat file containing the 3d model information

model_info_file = '3d_models/su57_3d_model.mat';

% define the reproduction speed factor
speedx = 0.5; 
% Do you want to save the animation in a mp4 file? (0.No, 1.Yes)
isave_movie = 0;
% Movie file name
movie_file_name = 'movie';

%% Run the simulation to get the data for the visualizer
load TP9.mat
out = run_simulation(3, TP9, x0, modelNonlinear, 2, 6, 40);

%% visusalizer
% -------------------------------------------------------------------------
% The frame sample time shall be higher than 0.02 seconds to be able to 
% update the figure (CPU/GPU constraints)
frame_sample_time = 0.01;

%% Run aircraft_3d_animation function
% -------------------------------------------------------------------------
close all
heading_deg = rad2deg(zeros(size(squeeze(out.state.Data(9,1,:)))));
pitch_deg = rad2deg(squeeze(out.state.Data(8,1,:)));
bank_deg = rad2deg(squeeze(out.state.Data(7,1,:)));
roll_command = zeros(size(heading_deg));
pitch_command = zeros(size(heading_deg));
angle_of_attack_deg = rad2deg(squeeze(out.state.Data(3,1,:)));
angle_of_sideslip_deg = rad2deg(squeeze(out.state.Data(2,1,:)));
fligh_path_angle_deg = pitch_deg - angle_of_attack_deg;
mach = 0.344 * ones(size(heading_deg));
altitude_ft = 25000 * ones(size(heading_deg));

nz_g = zeros(size(altitude_ft));
for i=1:1:length(heading_deg)
    coef = compute_coef(squeeze(out.logsout{1}.Values.Data(:,1,i)), squeeze(out.state.data(:,1,i)));
    dyn = compute_dyn_pressure(squeeze(out.state.data(:,1,i)));
    [forces, moments] = compute_forces_moments(coef, dyn);
    nz_g(i,1) = forces(2)/forces(3);
end

controls_deflection_deg = zeros(length(heading_deg),14);

aircraft_3d_animation(model_info_file,...
    heading_deg, ...            Heading angle [deg]
    pitch_deg, ...              Pitch angle [deg]
    bank_deg, ...               Roll angle [deg]
    roll_command, ...           Roll  stick command [-1,+1] [-1 -> left,            +1 -> right]
    pitch_command, ...          Pitch stick command [-1,+1] [-1 -> full-back stick, +1 -> full-fwd stick]
    angle_of_attack_deg, ...    AoA [deg]
    angle_of_sideslip_deg, ...  AoS [deg]
    fligh_path_angle_deg, ...   Flight path angle [deg]
    mach, ...                   Mach number
    altitude_ft, ...            Altitude [ft]
    nz_g,  ...                  Vertical load factor [g]
    controls_deflection_deg, ...Flight control deflection (each column is a control surface)
    frame_sample_time, ...      Sample time [sec]
    speedx, ...                 Reproduction speed
    isave_movie, ...            Save the movie? 0-1
    movie_file_name);           % Movie file name