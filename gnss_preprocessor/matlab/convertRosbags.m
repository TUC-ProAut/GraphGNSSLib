%% Convert the generated Rosbag file from GraphGNSSLib to a .mat file

clc 
close all
clear

%% Dataset selection
Dataset = 1;
%1: Berlin - Gendarmenmarkt
%2: Berlin - Potsdamer Platz
%3: Frankfurt - Main Tower
%4: Frankfurt - Westend Tower

%% load Rosbag GraphGNSSLib
if Dataset == 1
    Rosbag = rosbag('Berlin_Gendarmenmarkt_preprocessed.bag');
elseif Dataset == 2
    Rosbag = rosbag('Berlin_Potsdamer_Platz_preprocessed.bag');
elseif Dataset == 3
    Rosbag = rosbag('Frankfurt_Main_Tower_preprocessed.bag');
elseif Dataset == 4
    Rosbag = rosbag('Frankfurt_Westend_Tower_preprocessed.bag');
end

%% Bag conversion
Topic = select(Rosbag,'Topic','/gnss_preprocessor_node/GNSSPsrCarRov1');
ROS_struct = readMessages(Topic,'DataFormat','struct');

if Dataset == 1
    Data.Info.DatasetName = 'Berlin_Gendarmenmarkt';
elseif Dataset == 2
    Data.Info.DatasetName = 'Berlin_Potsdamer_Platz';
elseif Dataset == 3
    Data.Info.DatasetName = 'Frankfurt_Main_Tower';
elseif Dataset == 4
    Data.Info.DatasetName = 'Frankfurt_Westend_Tower';
end

Data.Info.TimeRef = ROS_struct{1}.GNSSRaws(1).GNSSTime; % absolute time reference

Data.GT_ClockError.Time = [];
Data.GT_ClockError.Mean = [];

Data.Pseudorange3.Time = [];
Data.Pseudorange3.Mean = [];
Data.Pseudorange3.Cov = [];
Data.Pseudorange3.SatPos = [];
Data.Pseudorange3.SatID = [];
Data.Pseudorange3.SatElevation = [];
Data.Pseudorange3.SNR = [];
Data.Pseudorange3.Bonus_MeanRaw = [];
%Data.Pseudorange3.Bonus_Error = [];
%Data.Pseudorange3.Bonus_GT = [];
%Data.Pseudorange3.Bonus_Sagnac = [];
Data.Pseudorange3.Bonus_Ionosphere = [];
Data.Pseudorange3.Bonus_Troposphere = [];
Data.Pseudorange3.Bonus_SatClockBias = [];
%Data.Pseudorange3.Bonus_NLOS = [];
%Data.Pseudorange3.Bonus_Clock = [];

for n = 1:length(ROS_struct)
    relTime = ROS_struct{n}.GNSSRaws(1).GNSSTime - Data.Info.TimeRef;
    Data.GT_ClockError.Time = [Data.GT_ClockError.Time ; relTime];
    Data.GT_ClockError.Mean = [Data.GT_ClockError.Mean ; 0];
    for sat = 1:length([ROS_struct{n}.GNSSRaws.PrnSatellitesIndex])
        Data.Pseudorange3.Time = [Data.Pseudorange3.Time ; ROS_struct{n}.GNSSRaws(sat).GNSSTime - Data.Info.TimeRef]; % generate relative time
        Data.Pseudorange3.Mean = [Data.Pseudorange3.Mean ; ROS_struct{n}.GNSSRaws(sat).Pseudorange];
        Data.Pseudorange3.Cov = [Data.Pseudorange3.Cov ; 0];
        Data.Pseudorange3.SatPos = [Data.Pseudorange3.SatPos ; ROS_struct{n}.GNSSRaws(sat).SatPosX ROS_struct{n}.GNSSRaws(sat).SatPosY ROS_struct{n}.GNSSRaws(sat).SatPosZ];
        Data.Pseudorange3.SatID = [Data.Pseudorange3.SatID ; ROS_struct{n}.GNSSRaws(sat).PrnSatellitesIndex];
        Data.Pseudorange3.SatElevation = [Data.Pseudorange3.SatElevation ; ROS_struct{n}.GNSSRaws(sat).Elevation];
        Data.Pseudorange3.SNR = [Data.Pseudorange3.SNR ; ROS_struct{n}.GNSSRaws(sat).Snr];
        Data.Pseudorange3.Bonus_MeanRaw = [Data.Pseudorange3.Bonus_MeanRaw ; ROS_struct{n}.GNSSRaws(sat).RawPseudorange];
        %Data.Pseudorange3.Bonus_Error = [Data.Pseudorange3.Bonus_Error ; 0];
        %Data.Pseudorange3.Bonus_GT = [Data.Pseudorange3.Bonus_GT ; 0];
        %Data.Pseudorange3.Bonus_Sagnac = [Data.Pseudorange3.Bonus_Sagnac ; 0];
        Data.Pseudorange3.Bonus_Ionosphere = [Data.Pseudorange3.Bonus_Ionosphere ; ROS_struct{n}.GNSSRaws(sat).ErrIono];
        Data.Pseudorange3.Bonus_Troposphere = [Data.Pseudorange3.Bonus_Troposphere ; ROS_struct{n}.GNSSRaws(sat).ErrTropo];
        Data.Pseudorange3.Bonus_SatClockBias = [Data.Pseudorange3.Bonus_SatClockBias ; ROS_struct{n}.GNSSRaws(sat).SatClkErr / 299792458];  % bias in seconds instead of meters
        %Data.Pseudorange3.Bonus_NLOS = [Data.Pseudorange3.Bonus_NLOS ; 0];
        %Data.Pseudorange3.Bonus_Clock = [Data.Pseudorange3.Bonus_Clock ; 0];
    end
end

if Dataset == 1
    save('Data_Berlin_Gendarmenmarkt_GraphGNSSLib.mat', 'Data')
elseif Dataset == 2
    save('Data_Berlin_Potsdamer_Platz_GraphGNSSLib.mat', 'Data')
elseif Dataset == 3
    save('Data_Frankfurt_Main_Tower_GraphGNSSLib.mat', 'Data')
elseif Dataset == 4
    save('Data_Frankfurt_Westend_Tower_GraphGNSSLib.mat', 'Data')
end