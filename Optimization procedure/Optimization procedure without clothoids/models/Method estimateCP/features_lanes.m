clc
clear all
close all

%% Load all mat files
mask_elem = '../../experimental-data/Images for calibration/estimateCP method dataset/';
kml_data_dir = '../../experimental-data/kml keypoints/';
image_dir = '../../experimental-data/Images for calibration/estimateCP method dataset/Test/';
video_dir = '../../experimental-data/';

dinfo = dir([mask_elem, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

dinfo = dir([video_dir, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

%% Which frames should I look to?

fps = 50;
n_frame_start = 154;
n_frame_end = 4163; % = 85;

idx_video = find(video.timesteps(:)>= n_frame_start*(1/fps) & ...
    video.timesteps(:)<n_frame_end*(1/fps));
lower_bound = 10.6;
upper_bound = 13.3;
idx_test = [];

for i=1:length(idx_video)
    if video.timesteps(idx_video(i))-video.timesteps(idx_video(1))>lower_bound...
            && video.timesteps(idx_video(i))-video.timesteps(idx_video(1))<upper_bound
        idx_test = [idx_test,idx_video(i)];
    end
end


%%

v = VideoReader([video_dir, 'videoredo2.mp4']);

cnt = 1;
found = 0;
while v.hasFrame
    read(v,cnt);
    
    t_frame = v.CurrentTime;
    
    if t_frame >= video.timesteps(idx_test(1)) && t_frame <= video.timesteps(idx_test(length(idx_test)))
        found = found + 1;
        frame = readFrame(v);
        frame_container(found).Image = frame;
        
    end
    cnt = cnt+1;
end


%% Sanity check

imshow(frame_container(20).Image)

%% Save images

for i=1:length(frame_container)
    imwrite(frame_container(i).Image,[image_dir,strcat('test',num2str(i),'.png')]);
end

%% Define ROI for line search

x_th = 1:size(frame,2);
y_th_low = 540;
y_th_up  = 290;

imshow(frame_container(20).Image)
hold on
plot(x_th,y_th_low,'r*')
plot(x_th,y_th_up, 'g*')

%% Let's filter the images to find the contours
masks_collector(length(frame_container))=struct();
%%
for i=1:length(frame_container)
    img = imread([image_dir,strcat('test',num2str(i),'.png')]);
    borders = edge(rgb2gray(img),'Canny',[0.16 0.32]);
    
    hold on
    masks_collector(i).Edges = borders;
    if i==1
        imshow(borders)
    end
end

% remove car borders
for i=1:length(frame_container)
    
    nocar_border = and(masks_collector(i).Edges,not(carmask_bw));
    masks_collector(i).Edges_nocar = nocar_border;
    imshow(nocar_border)
end

%%
close all
imshow(masks_collector(1).Edges_nocar)


%% Select the curve delimiters per each image

% In this section, and in particular for this code, it is relevant to
% define a region of interest with upper and lower bound within wich pixels
% are selected. So, let's define a policy for which white pixels whose y 
% value is y_lower_bound < y < y_upper_bound are kept equal to 1. Otherwise
% set them to 0.

for k=1:length(masks_collector)
    image_trial = masks_collector(k).Edges_nocar;
    
    for i=1:size(image_trial,1)
        for j=1:size(image_trial,2)
           if i<y_th_up || i>y_th_low
               image_trial(i,j) = 0;
           end
        end
    end
    
    masks_collector(k).Mask_ROI = image_trial;
end

%%

close all
imshow(masks_collector(130).Mask_ROI)

%% Collect left and right border

% Policy: divide the mask in left and right side; divide each side in upper
% and lower parts. Find the left border as the longest line in the bottom
% left part and the right border as the longest line in the bottom right
% part. Then, isolate the full detected lines and merge them together.

% Issues: this policy is not always working as some lines will be the
% external lane border or lanes inside the kerbs

for i=1:length(masks_collector)
    masks_collector(i).Borders = brd_identify(masks_collector(i).Mask_ROI,...
        y_th_up,y_th_low);
end

%%
close all
imshow(masks_collector(1).Borders)

%% Pixel list right and left border

% create ordered list of pixels from bottom to top

for k=1:length(masks_collector)
    k
    cc = bwconncomp(masks_collector(k).Borders,26);
        
    lines = regionprops(cc,"PixelList","PixelIdxList");
    for l=1:length(lines)
        array_2b_ord = lines(l).PixelList;
        [arr2bord_col,idx_col] = sort(array_2b_ord(:,2),'descend');
        array_ord = [array_2b_ord(idx_col,1),arr2bord_col];
        
        start_arr_idx = find(array_ord(:,2)==max(array_ord(:,2)));
        l
        if length(start_arr_idx) > 1
            if array_ord(start_arr_idx(1),1)<size(masks_collector(k).Borders,2)/2
                left_branch = array_ord;
                start_xval = find(left_branch(start_arr_idx)==min(left_branch(start_arr_idx)));
                start_point_left = left_branch(start_xval,:);
        
                % let's reorder
        
                new_branch = zeros(size(left_branch,1),2);
                reduced_branch = left_branch;
                
                for i=1:size(new_branch,1)
                    for j=1:2
                        if i==1
                            
                            new_branch(i,j) = start_point_left(i,j);
                            
                        else
                            if j == 1
                                qp = [new_branch(i-1,1) new_branch(i-1,2)];
                                [~,pop_idx] = ismember(qp,reduced_branch,'rows');
                                reduced_branch(pop_idx,:) = [];
                            end
                            new_branch(i,j) = reduced_branch(dsearchn(reduced_branch,qp),j);
                        end
                    end
                end
                left_branch = new_branch;
        
        
            else
                right_branch = array_ord;
                start_xval = find(right_branch(start_arr_idx)==max(right_branch(start_arr_idx)));
                start_point_right = right_branch(start_xval,:);
        
                % let's reorder
        
                new_branch = zeros(size(right_branch,1),2);
                reduced_branch = right_branch;
                
                for i=1:size(new_branch,1)
                    for j=1:2
                        if i==1
                            
                            new_branch(i,j) = start_point_right(i,j);
                            
                        else
                            if j == 1
                                qp = [new_branch(i-1,1) new_branch(i-1,2)];
                                [~,pop_idx] = ismember(qp,reduced_branch,'rows');
                                reduced_branch(pop_idx,:) = [];
                            end
                            new_branch(i,j) = reduced_branch(dsearchn(reduced_branch,qp),j);
                        end
                    end
                end
                right_branch = new_branch;
        
            end
        else
            if array_ord(start_arr_idx(1),:)<size(masks_collector(k).Borders,2)/2
                left_branch = array_ord;
                start_point_left = left_branch(start_arr_idx,:);
        
                % reorder
        
                new_branch = zeros(size(left_branch,1),2);
                reduced_branch = left_branch;
                
                for i=1:size(new_branch,1)
                    for j=1:2
                        if i==1
                            
                            new_branch(i,j) = start_point_left(i,j);
                            
                        else
                            if j == 1
                                qp = [new_branch(i-1,1) new_branch(i-1,2)];
                                [~,pop_idx] = ismember(qp,reduced_branch,'rows');
                                reduced_branch(pop_idx,:) = [];
                            end
                            new_branch(i,j) = reduced_branch(dsearchn(reduced_branch,qp),j);
                        end
                    end
                end
                left_branch = new_branch;
        
        
            else
                right_branch = array_ord;
                start_point_right = right_branch(start_arr_idx,:);
                
                % reorder
                new_branch = zeros(size(right_branch,1),2);
                reduced_branch = right_branch;
                
                for i=1:size(new_branch,1)
                    for j=1:2
                        if i==1
                            
                            new_branch(i,j) = start_point_right(i,j);
                            
                        else
                            if j == 1
                                qp = [new_branch(i-1,1) new_branch(i-1,2)];
                                [~,pop_idx] = ismember(qp,reduced_branch,'rows');
                                reduced_branch(pop_idx,:) = [];
                            end
                            new_branch(i,j) = reduced_branch(dsearchn(reduced_branch,qp),j);
                        end
                    end
                end
                right_branch = new_branch;
        
        
            end
        end
       
    end
    masks_collector(k).Left_border = left_branch;
    masks_collector(k).Right_border = right_branch;
end

close all
imshow(masks_collector(12).Borders)
hold on
plot(masks_collector(12).Left_border(3,1),masks_collector(12).Left_border(3,2),'go')
plot(masks_collector(12).Right_border(3,1),masks_collector(12).Right_border(3,2),'go')



%% Save the masks

dir_saving = '../../experimental-data/Images for calibration/estimateCP method dataset/';

save([dir_saving, 'image_row_data.mat'],"masks_collector");

