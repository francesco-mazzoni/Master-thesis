clc
clear all
close all

%% Camera params

% Try to estimate camera parameters using the calibration function in
% Matlab

set(0,'DefaultFigureWindowStyle','normal')
%% Load all mat files
data_dir = '../../experimental-data/Images for calibration/estimateCP method dataset/Chicane/';
kml_data_dir = '../../experimental-data/kml keypoints/';
dinfo = dir([data_dir, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

%% Eclude some points

EXCLUDE_PTS_FLAG = 0; % Set to 1 if you want to consider less than 28 points
EXCLUDE_IMG_FLAG = 0; % Set to 1 if you want to consider less than 22 images
RESTORE_FLAG = 1;       % Set to 1 if you want to restore all values to 'T'

%% Restore values

if RESTORE_FLAG == 1  

    for i=1:length(img_struct)

        % Restore all the "Include" to 'T' for the images
        if img_struct(i).Include == 'F'
            img_struct(i).Include = 'T';
        end

        % Restore all the "Include" to 'T' for the pixels
        for j=1:length(img_struct(i).Pixels)
            if img_struct(i).Pixels(j).Include == 'F'
                img_struct(i).Pixels(j).Include = 'T';
            end
        end

    end
    
    % Restore all the "Include" to 'T' for the world points
    for i=1:length(world_pts_struct)
        if world_pts_struct(i).Include == 'F'
            world_pts_struct(i).Include = 'T';
        end
    end

    % Restore all the "Include" to 'T' for the world points test 5
    for i=1:length(world_pts_struct_t2)
        if world_pts_struct_t2(i).Include == 'F'
            world_pts_struct_t2(i).Include = 'T';
        end
    end

    % Restore all the "Include" to 'T' for the world points test 8
    for i=1:length(world_pts_struct_t3)
        if world_pts_struct_t3(i).Include == 'F'
            world_pts_struct_t3(i).Include = 'T';
        end
    end
end

%% Define keypoints, image coordinates

% -------------------------- IMAGE SELECTION ------------------------------

% Here we can include all images or exclude the ones of interest

if EXCLUDE_IMG_FLAG == 1  
    exclude_img = [5,11]; % seem to be outliers according to pose plot after calibration
    for i=1:length(exclude_img)
        img_struct(exclude_img(i)).Include = 'F';
    end
else
    exclude_img = [];
end

% Consider the images to be included (Include = 'T')
NOTRUE = 0;                                      % number of 'T' elements
WTRUE = zeros(1,length(img_struct)-length(exclude_img)-length(exclude_img)); % location of 'T'

for i=1:length(img_struct)
    if img_struct(i).Include == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

% -------------------------- PIXEL SELECTION ------------------------------

NOPX = zeros(1,NOTRUE); % store how many pixels per image are available

% Include all pixels per image or exclude some of them (suppose that the
% initial number of pixels to be excluded is the same for all images)
% Set the following pixels to 'F': [9,10,11,12,17,18,27,28,29,30]
exclude_px = [9,10,11,12,17,18,27,28,29,30];
if EXCLUDE_PTS_FLAG == 1    
    for j=1:NOTRUE
        for i=1:length(exclude_px)
            img_struct(WTRUE(j)).Pixels(exclude_px(i)).Include = 'F';
        end
    end
end

% Consider the pixels to be included (typically I select them apart so that
% the number of pixel per image coincides)
for j=1:length(WTRUE)
    for i=1:length(img_struct(WTRUE(j)).Pixels)
        if img_struct(WTRUE(j)).Pixels(i).Include == 'T'
            NOPX(j) = NOPX(j) + 1;
        end
    end
end

% Where are the 'T' pixels to be included in terms of indeces in the struct?
px_loc = zeros(NOTRUE,max(NOPX));
cnt = 0;

for j=1:length(WTRUE)
    for i=1:length(img_struct(WTRUE(j)).Pixels)
        if img_struct(WTRUE(j)).Pixels(i).Include == 'T'
            cnt = cnt + 1;
            px_loc(j,cnt) = i;
        end
    end
    cnt = 0;
end

% --------------------- "imagePoints" MATRIX CREATION ---------------------

% Once the location is found create the imagePoints matrix based on the
% number of 'T' values and their locations

imagePoints = zeros(max(NOPX),2,length(WTRUE));

for i=1:length(WTRUE)
    for k=1:NOPX(i)
        imagePoints(k,1,i) = img_struct(WTRUE(i)).Pixels(px_loc(i,k)).Values(1);
        imagePoints(k,2,i) = img_struct(WTRUE(i)).Pixels(px_loc(i,k)).Values(2);
    end
end

%%
if 1
    for k = 1:size(imagePoints,3)
        imagename = strcat('test',num2str(WTRUE(k)),'.png');
        figure;
        imshow([data_dir, imagename])
        hold on;
        plot(imagePoints(:,1,k),imagePoints(:,2,k),'go');
        text(imagePoints(:,1,k)+2,imagePoints(:,2,k)+5,cellstr(num2str((1:size(imagePoints,1))')))
        drawnow

    end
end

%% Geodetic coordinates extraction

O = kml2struct([kml_data_dir, 'Origine traguardo.kml']);
grid  = kml2struct([kml_data_dir, 'grid.kml']);
% grid_test5  = kml2struct([kml_data_dir, 'grid_test5.kml']);
% grid_test8  = kml2struct([kml_data_dir, 'grid_test8.kml']);
chicane = kml2struct([kml_data_dir, 'chicane 1 v2.kml']);
kerb = kml2struct([kml_data_dir, 'keypoints cordolo def.kml']);
origin = [O.Lat, O.Lon, O.Alt];

%% Define input parameters, world coordinates

% ----------------------- WORLD POINTS SELECTION --------------------------

% Here we can exclude the points that correspond to the excluded pixels
exclude_pts = exclude_px;
if EXCLUDE_PTS_FLAG == 1
    
    for j=1:length(exclude_pts)
        world_pts_struct(exclude_pts(j)).Include = 'F';
    end
end

% Count the number of 'T' points to be included in the calibration
NOTRUE = 0;

for i=1:length(world_pts_struct)
    if world_pts_struct(i).Include == 'T'
        NOTRUE = NOTRUE + 1;
    end
end

% Save the location of these points
w_loc = zeros(1,NOTRUE);
cnt = 0;

for i=1:length(world_pts_struct)
    if world_pts_struct(i).Include == 'T'
        cnt = cnt + 1;
        w_loc(cnt) = i;
    end
end

% -------------------- "worldPoints" MATRIX CREATION ----------------------

worldPoints = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        worldPoints(i,j) = world_pts_struct(w_loc(i)).Value(j);
    end
end

%%

figure()
plot([worldPoints(:,1)],[worldPoints(:,2)],'*');
hold on
% plot(worldPoints(24,1),worldPoints(24,2),'go');
axis equal

%% Parameters estimation
I = imread([data_dir, 'test3.png']);
imageSize = [size(I, 1),size(I, 2)];

KGuess = [1500 0 imageSize(1)/2; ...
          0 1500 imageSize(2)/2; ...
          0 0 1];
% [params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,:),worldPoints, ...  
% [params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,[1:17,19:22]),worldPoints, ...  
% [params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,[1:8,11,13,15:17,19,21,22]),worldPoints, ...  % This is used with an origin in the 13th point instead of the 24th
% [params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,[1:8,10,11,13:22]),worldPoints, ...  
[params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,:),worldPoints, ...  
                                  'ImageSize',imageSize,'WorldUnits','m',...
                                  'NumRadialDistortionCoefficients',3,...
                                  'EstimateTangentialDistortion',false,...
                                  'EstimateSkew',false);%, ...
%                                   'InitialIntrinsicMatrix', KGuess');



% Store intrinsics parameters
intrinsics = params.Intrinsics;

%% Display params

figure
showReprojectionErrors(params);
drawnow
figure;
showExtrinsics(params,"PatternCentric");
% showExtrinsics(params,"CameraCentric");
axis equal
drawnow

displayErrors(errors,params);

%%

figure()
plot(params.WorldPoints(:,1), params.WorldPoints(:,2),'go')
hold on
text(params.WorldPoints(:,1)+0.25,params.WorldPoints(:,2)+0.25,cellstr(num2str((1:size(params.WorldPoints,1))')))
for k=1:size(imagePoints,3)
    plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(k)),Color="blue");
    scatter3(params.WorldPoints(params.DetectedKeypoints(:,k),1), params.WorldPoints(params.DetectedKeypoints(:,k)',2),0,20,'g','filled')
    tmp_pose = extr2pose(params.PatternExtrinsics(k));
    tmp_xyz = tmp_pose.Translation;
    scatter3(tmp_xyz(1),tmp_xyz(2),20,'b','filled')
end
axis equal
title("Camera positions")


%% Plot calibration image and calibration poses


% Notice: this works only once after the code has been run from the
% beginning in order. If one goes back after the single test for camera
% pose estimation WTRUE will be different and the lines below will show a
% complitely different image from the one expected

% CAMBIARE NOME ALLA VARIABILE WTRUE USATA IN FASE DI VALIDAZIONE PER
% EVITARE IL PROBLEMA?

for k = 1:size(imagePoints,3)
    imagename = strcat('test',num2str(WTRUE(k)),'.png');
    figure;

%     subplot(1,2,1)
    imshow([data_dir, imagename])
    hold on;
    plot(imagePoints(:,1,k),imagePoints(:,2,k),'go');
    plot(params.ReprojectedPoints(:,1,k),params.ReprojectedPoints(:,2,k),'r+');
    legend('Detected Points','ReprojectedPoints');
    hold off;
    drawnow;
    title("Image file:" + strcat('test',num2str(WTRUE(k)),'.png'));

%     subplot(1,2,2)
    figure;
    hold on;
    plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(k)),Color="blue");
    scatter3(params.WorldPoints(:,1), params.WorldPoints(:,2),zeros(size(params.WorldPoints,1),1),20,'r','filled')
    scatter3(params.WorldPoints(params.DetectedKeypoints(:,k),1), params.WorldPoints(params.DetectedKeypoints(:,k)',2),0,20,'g','filled')
    text(params.WorldPoints(:,1)+0.25,params.WorldPoints(:,2)+0.25,cellstr(num2str((1:size(params.WorldPoints,1))')))
    tmp_pose = extr2pose(params.PatternExtrinsics(k));
    tmp_xyz = tmp_pose.Translation;
    scatter3(tmp_xyz(1),tmp_xyz(2),tmp_xyz(3),20,'b','filled')
    axis equal
    title("Calibration frame # " + num2str(k))

    drawnow;
end


%% Find position of camera for the frame in test2.png 

exclude = exclude_pts;
if EXCLUDE_PTS_FLAG == 1
    for j=1:length(exclude)
        world_pts_struct_t2(exclude(j)).Include = 'F';
    end
end

% ------------------------------------------------------------------------- 
% Check if some pixels are [NaN,NaN]. We can't use them for camera pose estimation.
% If some NaNs are present, reorder the points with only valid values and 
% exclude the corresponding worldPoints

% Set to 'False'
nanc = 0;
for i=1:length(img_struct(2).Pixels)
    if isnan(img_struct(2).Pixels(i).Values)
        nanc = nanc + 1;
        img_struct(2).Pixels(i).Include = 'F';
    end
end

% Check where are the points to be included
NOTRUE = 0;
if EXCLUDE_PTS_FLAG == 1
    WTRUE = zeros(1,length(img_struct(2).Pixels)-length(exclude)-nanc);
else
    WTRUE = zeros(1,length(img_struct(2).Pixels)-nanc);
end

for i=1:length(img_struct(2).Pixels)
    if img_struct(2).Pixels(i).Include == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

% ---------- New "worldPoints" MATRIX FOR CAMERA POSE ESTIMATION ----------

worldPointsGlobal_test2 = zeros(length(world_pts_struct_t2),2);

for i=1:length(world_pts_struct_t2)
    for j=1:2
        worldPointsGlobal_test2(i,j) = world_pts_struct_t2(i).Value(j);
    end
end

% ---------- New "imagePoints" MATRIX FOR CAMERA POSE ESTIMATION ----------

% the next lines of code are used to augment the number of keypoints. In
% the dataset_extraction.m file the number of worldPoints is augmented by
% considering the mean positions between keypoints of the shorter kerb
% segment. The corresponding pixels are added here.

imagePoints_test2 = zeros(length(world_pts_struct_t2),2,1);
cnt = 2;
for i=1:length(world_pts_struct_t2)
    if i < 17 % 17 is the starting extra point
        imagePoints_test2(i,1,1) = img_struct(2).Pixels(WTRUE(i)).Values(1);
        imagePoints_test2(i,2,1) = img_struct(2).Pixels(WTRUE(i)).Values(2);
    else
        imagePoints_test2(i,1,1) = (img_struct(2).Pixels(WTRUE(cnt)).Values(1)+img_struct(2).Pixels(WTRUE(cnt+1)).Values(1))/2;
        imagePoints_test2(i,2,1) = (img_struct(2).Pixels(WTRUE(cnt)).Values(2)+img_struct(2).Pixels(WTRUE(cnt+1)).Values(2))/2;
        cnt = cnt + 2;
    end
end

figure; 
imshow([data_dir, 'test2.png'])
hold on;
plot(imagePoints_test2(:,1,1),imagePoints_test2(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,2),params.ReprojectedPoints(:,2,2),'r+');
text(imagePoints_test2(:,1,1)+2,imagePoints_test2(:,2,1)+5,cellstr(num2str((1:size(imagePoints_test2,1))')))
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;


% Calculate camera extrinsics
% The extrinsics are a transformation from world coordinates to camera coordinates 
% that enables you to transform points from the world coordinate system to the camera coordinate system.
% camExtrinsics = estimateExtrinsics(imagePoints_test2,worldPointsGlobal_test2,intrinsics);

% NB: camExtrinsics is obtained by considering the first group of
% keypoints, i.e. the one closer to the sensor. This is done to see if the
% pose estimation based on many pixels, not far away from the camera, is
% better than considering more distant keypoints (seems to be so).
% Please consider the following: from 1 to 16 consider the original
% keypoints. The ones up to 11 are the nearest. From 17 to 21 we consider
% the extra set of points for the nearest pattern. The 22nd and 23rd point
% refer to the furthest pattern.

camExtrinsics = estimateExtrinsics(imagePoints_test2([1:11,17:21],:),worldPointsGlobal_test2([1:11,17:21],:),intrinsics);


% Inverse transformation of the above (can be also obtained by doing inv(camExtrinsics.A)) 
cameraPose = extr2pose(camExtrinsics);

% This translation is relative to the origin of the set of world points we are
% using, orign that we set on the first point of the set

xyz = cameraPose.Translation; % extract translation

% To check thi rotation
eul = rotm2eul(cameraPose.R); % default rotation is ZYX

% plot the pose with all the keypoints, even if some of them are not used
% for the cameraExtrinsics calculation

figure;
hold on;
plotCamera(AbsolutePose = cameraPose);
scatter3(worldPointsGlobal_test2(:,1), worldPointsGlobal_test2(:,2),zeros(size(worldPointsGlobal_test2,1),1),20,'g','filled')
scatter3(worldPointsGlobal_test2(5,1), worldPointsGlobal_test2(5,2),0,20,'r','filled')
text(worldPointsGlobal_test2(:,1)+0.25,worldPointsGlobal_test2(:,2)+0.25,cellstr(num2str((1:size(worldPointsGlobal_test2,1))')))
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
axis equal

% Find absolute position of point 1 of the set w.r.t. origin
[p2x_abs,p2y_abs,p2z_abs] = latlon2local(chicane(5).Lat,chicane(5).Lon,chicane(5).Alt,origin);
p2_xyz_abs = [p2x_abs,p2y_abs,p2z_abs];

cam_pos_test2 = p2_xyz_abs + xyz; % absolute griund positon of the camera 


%% Find position of camera for the frame in test13.png 

exclude = exclude_pts;
if EXCLUDE_PTS_FLAG == 1
    for j=1:length(exclude)
        world_pts_struct_t3(exclude(j)).Include = 'F';
    end
end

% ------------------------------------------------------------------------- 
% Check if some pixels are [NaN,NaN]. We can't use them for camera pose estimation.
% If some NaNs are present, reorder the points with only valid values and 
% exclude the corresponding worldPoints

% Set to 'False'
nanc = 0;
for i=1:length(img_struct(13).Pixels)
    if isnan(img_struct(13).Pixels(i).Values)
        nanc = nanc + 1;
        img_struct(13).Pixels(i).Include = 'F';
    end
end

% Check where are the points to be included
NOTRUE = 0;
WTRUE = zeros(1,length(img_struct(13).Pixels)-nanc);

for i=1:length(img_struct(13).Pixels)
    if img_struct(13).Pixels(i).Include == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

% ---------- New "worldPoints" MATRIX FOR CAMERA POSE ESTIMATION ----------

worldPointsGlobal_test3 = zeros(length(world_pts_struct_t3),2);

for i=1:length(world_pts_struct_t3)
    for j=1:2
        worldPointsGlobal_test3(i,j) = world_pts_struct_t3(i).Value(j);
    end
end

% ---------- New "imagePoints" MATRIX FOR CAMERA POSE ESTIMATION ----------

% the next lines of code are used to augment the number of keypoints. In
% the dataset_extraction.m file the number of worldPoints is augmented by
% considering the mean positions between keypoints of the shorter kerb
% segment. The corresponding pixels are added here.



imagePoints_test3 = zeros(length(world_pts_struct_t3),2,1);

cnt = 1;
for i=1:length(world_pts_struct_t3)
    if i < 11 % 17 is the starting extra point
        imagePoints_test3(i,1,1) = img_struct(13).Pixels(WTRUE(i)).Values(1);
        imagePoints_test3(i,2,1) = img_struct(13).Pixels(WTRUE(i)).Values(2);
    else
        imagePoints_test3(i,1,1) = (img_struct(13).Pixels(WTRUE(cnt)).Values(1)+img_struct(13).Pixels(WTRUE(cnt+1)).Values(1))/2;
        imagePoints_test3(i,2,1) = (img_struct(13).Pixels(WTRUE(cnt)).Values(2)+img_struct(13).Pixels(WTRUE(cnt+1)).Values(2))/2;
        cnt = cnt + 2;
    end
end


figure; 
imshow([data_dir, 'test13.png'])
hold on;
plot(imagePoints_test3(:,1,1),imagePoints_test3(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,11),params.ReprojectedPoints(:,2,11),'r+');
text(imagePoints_test3(:,1)+0.25,imagePoints_test3(:,2)+0.25,cellstr(num2str((1:size(imagePoints_test3,1))')))
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;

% Calculate camera extrinsics
% The extrinsics are a transformation from world coordinates to camera coordinates 
% that enables you to transform points from the world coordinate system to the camera coordinate system.

% NB: camExtrinsics is obtained by considering the first group of
% keypoints, i.e. the one closer to the sensor. This is done to see if the
% pose estimation based on many pixels, not far away from the camera, is
% better than considering more distant keypoints (differently from before, 
% things seem to become worse).
% Please consider the following: from 1 to 10 consider the original
% keypoints. From 11 to 15 we consider the extra set of points for the 
% nearest pattern

camExtrinsics = estimateExtrinsics(imagePoints_test3([1:10],:),worldPointsGlobal_test3([1:10],:),intrinsics);

% Inverse transformation of the above (can be also obtained by doing inv(camExtrinsics.A)) 
cameraPose = extr2pose(camExtrinsics);

% This translation is relative to the origin of the set of world points we are
% using, orign that we set on the first point of the set

xyz = cameraPose.Translation; % extract translation

% To check thi rotation
eul = rotm2eul(cameraPose.R); % default rotation is ZYX

figure;
hold on;
plotCamera(AbsolutePose = cameraPose);
scatter3(worldPointsGlobal_test3(:,1), worldPointsGlobal_test3(:,2),zeros(size(worldPointsGlobal_test3,1),1),20,'g','filled')
scatter3(worldPointsGlobal_test3(5,1), worldPointsGlobal_test3(5,2),0,20,'r','filled')
text(worldPointsGlobal_test3(:,1)+0.25,worldPointsGlobal_test3(:,2)+0.25,cellstr(num2str((1:size(worldPointsGlobal_test3,1))')))
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
axis equal

% Find absolute position of point 1 of the set w.r.t. origin
[p3x_abs,p3y_abs,p3z_abs] = latlon2local(chicane(13).Lat,chicane(13).Lon,chicane(13).Alt,origin);
p3_xyz_abs = [p3x_abs,p3y_abs,p3z_abs];

cam_pos_test3 = p3_xyz_abs + xyz; % absolute griund positon of the camera

% %% Kerb
% 
% % Check where are the points to be included
% NOTRUE = 0;
% WTRUE = [];
% 
% for i=1:length(world_pts_kerb)
%     if world_pts_kerb(i).Include == 'T'
%         NOTRUE = NOTRUE + 1;
%         WTRUE(NOTRUE) = i;
%     end
% end
% 
% % Create the worldPoints variable
% 
% worldPointsGlobal_kerb = zeros(NOTRUE,2);
% 
% for i=1:NOTRUE
%     for j=1:2
%         worldPointsGlobal_kerb(i,j) = world_pts_kerb(WTRUE(i)).Value(j);
%     end
% end
% 
% % Create the image Points variable
% 
% imagePoints_kerb = zeros(NOTRUE,2,1);
% 
% for i=1:NOTRUE
%     for j=1:2
%         imagePoints_kerb(i,j) = pixel_values_k(WTRUE(i)).Values(j);
%     end
% end
% 
% figure; 
% imshow([data_dir, 'test1_kerb.png'])
% hold on;
% plot(imagePoints_kerb(:,1),imagePoints_kerb(:,2),'go');
% % plot(params.ReprojectedPoints(:,1,8),params.ReprojectedPoints(:,2,8),'r+');
% % legend('Detected Points','ReprojectedPoints');
% hold off;
% drawnow;
% 
% camExtrinsics = estimateExtrinsics(imagePoints_kerb,worldPointsGlobal_kerb,intrinsics);
% 
% cameraPose = extr2pose(camExtrinsics);
% 
% xyz = cameraPose.Translation; % extract translation
% 
% eul = rotm2eul(cameraPose.R); % default rotation is ZYX
% 
% figure;
% hold on;
% scatter3(worldPointsGlobal_kerb(:,1), worldPointsGlobal_kerb(:,2),zeros(size(worldPointsGlobal_kerb,1),1),20,'g','filled')
% scatter3(worldPointsGlobal_kerb(5,1), worldPointsGlobal_kerb(5,2),0,20,'r','filled')
% scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
% axis equal
% 
% % Find absolute position of point 1 of the set w.r.t. origin
% [pg_1x_abs,pg_1y_abs,pg_1z_abs] = latlon2local(kerb(5).Lat,kerb(5).Lon,kerb(5).Alt,origin);
% pg_1_xyz_abs = [pg_1x_abs,pg_1y_abs,pg_1z_abs];
% 
% cam_pos_test_k = pg_1_xyz_abs + xyz; % absolute griund positon of the camera


%% Extract camera lat and long

[lat1,lon1,alt1] = local2latlon( cam_pos_test2(1), ...
    cam_pos_test2(2), ...
    cam_pos_test2(3), ...
    origin);


[lat2,lon2,alt2] = local2latlon( cam_pos_test3(1), ...
    cam_pos_test3(2), ...
    cam_pos_test3(3), ...
    origin);


% [lat3,lon3,alt3] = local2latlon( cam_pos_test_k(1), ...
%     cam_pos_test_k(2), ...
%     cam_pos_test_k(3), ...
%     origin);

%% export camera positions in kml file

kmlwritepoint('cp_chicane.kml',[lat1,lat2],[lon1,lon2], ...
    [alt1,alt2],'Name',{'Cam pos test image 2', 'Cam pos test image 13'});

%% Back projection trial

% from image plane to real world coordinate system

image_point_16 = [reproj_point_16.Position(1);reproj_point_16.Position(2); ...
    1];

camPoint_comps_16 = inv(params.K)*image_point_16;

worldpoint_proj_16 = inv(params.PatternExtrinsics(16).A)*[camPoint_comps_16(1);...
    camPoint_comps_16(2); camPoint_comps_16(3); 1];

figure
plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(16)),Color="blue");
hold on
scatter3(params.WorldPoints(:,1), params.WorldPoints(:,2),zeros(size(params.WorldPoints,1),1),20,'r','filled')
scatter3(params.WorldPoints(params.DetectedKeypoints(:,16),1), params.WorldPoints(params.DetectedKeypoints(:,16)',2),0,20,'g','filled')
text(params.WorldPoints(:,1)+0.25,params.WorldPoints(:,2)+0.25,cellstr(num2str((1:size(params.WorldPoints,1))')))
tmp_pose = extr2pose(params.PatternExtrinsics(16));
tmp_xyz = tmp_pose.Translation;
scatter3(tmp_xyz(1),tmp_xyz(2),tmp_xyz(3),20,'b','filled')
scatter3(worldpoint_proj_16(1),worldpoint_proj_16(2),worldpoint_proj_16(3),'black','filled')
axis equal
title("Calibration frame # " + num2str(16))

%% 

image_point_7 = [0.2*reproj_point_7.Position(1);0.2*reproj_point_7.Position(2); ...
    0.2];

camPoint_comps_7 = params.K\image_point_7;

worldpoint_proj_7 = params.PatternExtrinsics(7).A\[camPoint_comps_7(1);...
    camPoint_comps_7(2); camPoint_comps_7(3); 1];

figure
plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(7)),Color="blue");
hold on
scatter3(params.WorldPoints(:,1), params.WorldPoints(:,2),zeros(size(params.WorldPoints,1),1),20,'r','filled')
scatter3(params.WorldPoints(params.DetectedKeypoints(:,7),1), params.WorldPoints(params.DetectedKeypoints(:,7)',2),0,20,'g','filled')
text(params.WorldPoints(:,1)+0.25,params.WorldPoints(:,2)+0.25,cellstr(num2str((1:size(params.WorldPoints,1))')))
tmp_pose = extr2pose(params.PatternExtrinsics(7));
tmp_xyz = tmp_pose.Translation;
scatter3(tmp_xyz(1),tmp_xyz(2),tmp_xyz(3),20,'b','filled')
scatter3(worldpoint_proj_7(1),worldpoint_proj_7(2),worldpoint_proj_7(3),'black','filled')
axis equal
title("Calibration frame # " + num2str(7))