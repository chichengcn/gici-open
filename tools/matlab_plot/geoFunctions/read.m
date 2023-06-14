% read the binary data and convert them into .mat
clear; clc; close all;
% ------------ base obs ------------
fid = fopen('C:\Users\Administrator\Desktop\’≈–¿\LAB\softwareReceiver_cc\data\BaseL1L2.obs', 'rb');
Data_BaseL1L2 = [];
while (~feof(fid))
    satellite_PRN       = fread(fid, 1, 'double');
    GPS_obs_time        = fread(fid, 1, 'double');
    CA_code_pseudorange = fread(fid, 1, 'double');
    L1_carrier_phase    = fread(fid, 1, 'double');
    L1_Doppler          = fread(fid, 1, 'double');
    L2_carrier_phase    = fread(fid, 1, 'double');
    temp_Data = [satellite_PRN;
                 GPS_obs_time;
                 CA_code_pseudorange;
                 L1_carrier_phase;
                 L1_Doppler;
                 L2_carrier_phase];
    Data_BaseL1L2 = [Data_BaseL1L2, temp_Data];
end
save('data\Data_BaseL1L2');
%%
clear; clc; close;
% ------------ rover obs ------------
fid = fopen('C:\Users\Administrator\Desktop\’≈–¿\LAB\softwareReceiver_cc\data\RemoteL1L2.obs', 'rb');
Data_RemoteL1L2 = [];
while (~feof(fid))
    satellite_PRN       = fread(fid, 1, 'double');
    GPS_obs_time        = fread(fid, 1, 'double');
    CA_code_pseudorange = fread(fid, 1, 'double');
    L1_carrier_phase    = fread(fid, 1, 'double');
    L1_Doppler          = fread(fid, 1, 'double');
    L2_carrier_phase    = fread(fid, 1, 'double');
    temp_Data = [satellite_PRN;
                 GPS_obs_time;
                 CA_code_pseudorange;
                 L1_carrier_phase;
                 L1_Doppler;
                 L2_carrier_phase];
    Data_RemoteL1L2 = [Data_RemoteL1L2, temp_Data];
end
save('data\Data_RemoteL1L2');
%%
clear; clc; close;
% ------------ Satellite true position & velocity ------------
fid = fopen('C:\Users\Administrator\Desktop\’≈–¿\LAB\softwareReceiver_cc\data\Satellites.sat', 'rb');
Data_Satellites = [];
while (~feof(fid))
    satellite_PRN = fread(fid, 1, 'double');
    GPS_obs_time  = fread(fid, 1, 'double');
    position_X = fread(fid, 1, 'double');
    position_Y = fread(fid, 1, 'double');
    position_Z = fread(fid, 1, 'double');
    velocity_X = fread(fid, 1, 'double');
    velocity_Y = fread(fid, 1, 'double');
    velocity_Z = fread(fid, 1, 'double');
    temp_Data = [satellite_PRN;
                 GPS_obs_time;
                 position_X;
                 position_Y;
                 position_Z;
                 velocity_X;
                 velocity_Y;
                 velocity_Z];
    Data_Satellites = [Data_Satellites, temp_Data];
end
save('data\Data_Satellites');
