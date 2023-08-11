clc
clear all
close all

%% Video time-step vector extractor

% Define directory for extraction

data_dir = '../../experimental-data/';

%% Read video

v = VideoReader([data_dir, 'videoredo2.mp4']);

%% Create a vector containing all the time-steps per frame
cnt=1;
t_frame_container = zeros(1,v.NumFrames);
v_frame_contatiner = zeros(1,v.NumFrames);
while v.hasFrame
    read(v,cnt);
    
    t_frame = v.CurrentTime;

    if t_frame == 154*0.02
%         fprintf('4155*0.02')
        frame = readFrame(v);
        figure()
        imshow(frame)
        
    end

    t_frame_container(cnt) = t_frame;
    cnt=cnt+1;
end

video.timesteps = t_frame_container;

%% Save the vector in a struct to be used in parameters estimation algorithm

save_directory = '../../experimental-data/';

save([save_directory,'video.mat'],"video");