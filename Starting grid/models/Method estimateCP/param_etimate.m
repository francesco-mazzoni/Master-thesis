clc
clear all
close all

%% Camera params

% Try to estimate camera parameters using the calibration function in
% Matlab; then compare the results with the one obtained


%% Load all mat files
data_dir = '../../experimental-data/Images for calibration/estimateCP method dataset/3 and 2 starting positions/';
kml_data_dir = '../../experimental-data/kml keypoints/';
dinfo = dir([data_dir, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

resettrue = 1;
%% Define keypoints, image coordinates

NOTRUE = [];

% Consider the images to be included (Include = 'T')
for i=1:length(img_struct)
    if img_struct(i).Include(1) == 'T'
        NOTRUE = [NOTRUE,i];
    end
end

NOPX = zeros(1,length(NOTRUE)); % store how many pixels per image are available

% Set the following pixels to F
setf = [13:21,23:25,27:29];
for j=1:length(NOPX)
    for i=1:length(setf)
        img_struct(j).Pixels(setf(i)).Include(1) = 'F';
    end
end

if resettrue == 1
    for j=1:length(NOPX)
        for i=1:length(setf)
            img_struct(j).Pixels(setf(i)).Include(1) = 'T';
        end
    end
end

%%

% build the imagePoints matrix. First define the max number of true pixels
% visible in an image
counter = 0;
maxsize = 0;
for j=1:length(NOPX)
    for i=1:length(img_struct(NOTRUE(1)).Pixels)
        if img_struct(j).Pixels(i).Include(1) == 'T'
            counter = counter +1;
        end
    end
    if counter>maxsize
        maxsize = counter;
    end
    counter = 0;
end
imagePoints = zeros(maxsize,2,length(NOTRUE));

%%
% Consider the pixels to be included (typically I select them apart so that ...
% the number of pixel per image coincides)
where_px(length(NOTRUE))=struct();
%%
for j=1:length(NOTRUE)
    for i=1:length(img_struct(NOTRUE(j)).Pixels)
        if img_struct(j).Pixels(i).Include(1) == 'T'
            NOPX(j) = NOPX(j) + 1;
        end
    end
    where_px(j).true_px = zeros(NOPX(j),1);
end
for j=1:length(NOTRUE)
    slc = [];
    for i=1:length(img_struct(j).Pixels)
        if img_struct(j).Pixels(i).Include(1) == 'T'
            
            slc = [slc,i];
        end
    end
    for i=1:length(slc)
        where_px(j).true_px(i) = slc(i);
    end
end
% imagePoints = zeros(length(NOPX(1)),2,length(NOTRUE));

for i=1:length(NOTRUE)
    for k=1:length(where_px(i).true_px)
        if where_px(i).true_px(k)~=0
            imagePoints(k,1,i) = img_struct(NOTRUE(i)).Pixels(where_px(i).true_px(k)).Values(1);
            imagePoints(k,2,i) = img_struct(NOTRUE(i)).Pixels(where_px(i).true_px(k)).Values(2);
        else
            imagePoints(k,1,i) = NaN;
            imagePoints(k,2,i) = NaN;
        end
    end
    
end

%%
close all
figure; 
imshow([data_dir, 'test10.png'])
hold on;
plot(imagePoints(:,1,10),imagePoints(:,2,10),'go');
for i=1:size(imagePoints,1)
    text(imagePoints(i,1,10)+2,imagePoints(i,2,10)+2,num2str(i))
end
drawnow

%% Geodetic coordinates extraction

O = kml2struct([kml_data_dir, 'Origine traguardo.kml']);
grid  = kml2struct([kml_data_dir, 'grid.kml']);
grid_test5  = kml2struct([kml_data_dir, 'grid_test5.kml']);
grid_test8  = kml2struct([kml_data_dir, 'grid_test8.kml']);
kerb = kml2struct([kml_data_dir, 'keypoints cordolo def.kml']);
origin = [O.Lat, O.Lon, O.Alt];

% origin = [45.629828413016030, 9.291562998765919, 190.8882878615457];

%% Define input parameters, world coordinates

NOTRUE = 0;

for i=1:length(world_pts_struct)
    if world_pts_struct(i).Include(1) == 'T'
        NOTRUE = NOTRUE + 1;
    end
end

worldPoints = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        if world_pts_struct(i).Include(1) == 'T'
            worldPoints(i,j) = world_pts_struct(i).Value(j);
        end
    end
end
% worldPoints = worldPoints([1:12,22,26,30],:);
%%

figure()
plot([worldPoints(:,1)],[worldPoints(:,2)],'*');
hold on
plot(worldPoints(5,1),worldPoints(5,2),'go');
axis equal

%% Parameters estimation

I = imread([data_dir, 'test3.png']);
imageSize = [size(I, 1),size(I, 2)];

[params, imagesUsed, errors] = estimateCameraParameters(imagePoints(:,:,:),worldPoints, ...  
                                  'ImageSize',imageSize,'WorldUnits','m',...
                                  'NumRadialDistortionCoefficients',3,...
                                  'EstimateSkew',false,...
                                  'EstimateTangentialDistortion',true);


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

figure; 
imshow([data_dir, 'test5.png'])
hold on;
plot(imagePoints(:,1,5),imagePoints(:,2,5),'go');
plot(params.ReprojectedPoints(:,1,5),params.ReprojectedPoints(:,2,5),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;

figure; 
imshow([data_dir, 'test2.png'])
hold on;
plot(imagePoints(:,1,2),imagePoints(:,2,2),'go');
plot(params.ReprojectedPoints(:,1,2),params.ReprojectedPoints(:,2,2),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;


%% Find position of camera for the frame in test5.png 
% We have ABSOLUTE ground coordinates of only testa.png and teste.png 
%
% the points are stored in kerb1. ATTENTION: THIS WORKS FOR EVERY SET OF
% WORLD-IMAGE POINTS, IT DOES NOT NEED TO BE A KERBS

% we want ground coordinates, thus we transform geodetic to cartesian
% coordinates w.r.t. to the first point of the set

% Origin of the world points must be in the picture frame otherwise we lose
% calibration (calibration works well when it is well scaled in terms of cartesian world coordinates)

% --------------- 
% Check if some pixels are [NaN,NaN]. If so, reorder them with only valid
% values and exclude the corresponding worldPoints


% Set to 'False'
for i=1:length(imagePoints(:,:,5))
    if isnan(imagePoints(i,:,5))
        world_pts_struct_t5(i).Include = 'F';
        img_struct(5).Pixels(i).Include = 'F';
    end
end

% Check where are the points to be included
NOTRUE = 0;
WTRUE = [];

for i=1:length(worldPoints)
    if world_pts_struct_t5(i).Include(1) == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

worldPointsGlobal_test5 = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        worldPointsGlobal_test5(i,j) = world_pts_struct_t5(WTRUE(i)).Value(j);
    end
end


% Image points: select the ones that are not NaN

imagePoints_test5 = zeros(NOTRUE,2,1);

for i=1:NOTRUE
    imagePoints_test5(i,1,1) = img_struct(5).Pixels(WTRUE(i)).Values(1);
    imagePoints_test5(i,2,1) = img_struct(5).Pixels(WTRUE(i)).Values(2);
end
 
% Create the worldPoints variable

NOTRUE = 0;

for i=1:30
    if world_pts_struct(i).Include
        NOTRUE = NOTRUE + 1;
    end
end

worldPoints = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        if world_pts_struct(i).Include
            worldPointsGlobal_test5(i,j) = world_pts_struct_t5(WTRUE(i)).Value(j);
        end
    end
end

% Image points

imagePoints_test5 = imagePoints(:,:,5);

% Calculate camera extrinsics
% The extrinsics are a transformation from world coordinates to camera coordinates 
% that enables you to transform points from the world coordinate system to the camera coordinate system.
camExtrinsics = estimateExtrinsics(imagePoints_test5,worldPointsGlobal_test5,intrinsics);

% Inverse transformation of the above (can be also obtained by doing inv(camExtrinsics.A)) 
cameraPose = extr2pose(camExtrinsics);

% This translation is relative to the origin of the set of world points we are
% using, orign that we set on the first point of the set

xyz = cameraPose.Translation; % extract translation

% To check thi rotation
eul = rotm2eul(cameraPose.R); % default rotation is ZYX

figure;
subplot(1,3,[1 2])
imshow([data_dir, 'test5.png'])
hold on;
plot(imagePoints(:,1,5),imagePoints(:,2,5),'go');
plot(params.ReprojectedPoints(:,1,5),params.ReprojectedPoints(:,2,5),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;
% title("Image file:" + strcat('test',num2str(WTRUE(k)),'.png'));
subplot(1,3,3)
hold on;
scatter3(worldPointsGlobal_test5(:,1), worldPointsGlobal_test5(:,2),zeros(size(worldPointsGlobal_test5,1),1),20,'g','filled')
scatter3(worldPointsGlobal_test5(5,1), worldPointsGlobal_test5(5,2),0,20,'r','filled')
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(5)),Color="blue");
axis equal



% Find absolute position of point 1 of the set w.r.t. origin
[p5_1x_abs,p5_1y_abs,p5_1z_abs] = latlon2local(grid_test5(5).Lat,grid_test5(5).Lon,grid_test5(5).Alt,origin);
p5_1_xyz_abs = [p5_1x_abs,p5_1y_abs,p5_1z_abs];

cam_pos_test5 = p5_1_xyz_abs + xyz; % absolute griund positon of the camera 


%%

for k = 1:size(imagePoints,3)
    imagename = strcat('test',num2str(WTRUE(k)),'.png');
    figure;

%     subplot(2,3,[1 2 4 5])
    figure;
    imshow([data_dir, imagename])
    hold on;
    plot(imagePoints(:,1,k),imagePoints(:,2,k),'go');
    plot(params.ReprojectedPoints(:,1,k),params.ReprojectedPoints(:,2,k),'r+');
    legend('Detected Points','ReprojectedPoints');
    hold off;
    drawnow;
    title("Image file:" + strcat('test',num2str(WTRUE(k)),'.png'));

%     subplot(2,3,[3 6])
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



%% Find position of camera for the frame in test8.png 
% We have ABSOLUTE ground coordinates of only test5.png and test8.png 
%
% the points are stored in kerb1. ATTENTION: THIS WORKS FOR EVERY SET OF
% WORLD-IMAGE POINTS, IT DOES NOT NEED TO BE A KERBS

% we want ground coordinates, thus we transform geodetic to cartesian
% coordinates w.r.t. to the first point of the set

% Origin of the world points must be in the picture frame otherwise we lose
% calibration (calibration works well when it is well scaled in terms of cartesian world coordinates)

% --------------- 
% Check if some pixels are [NaN,NaN]. If so, reorder them with only valid
% values and exclude the corresponding worldPoints


% Set to 'False'
for i=1:length(imagePoints(:,:,8))
    if isnan(imagePoints(i,:,8))
        world_pts_struct_t8(i).Include = 'F';
        img_struct(8).Pixels(i).Include = 'F';
    end
end

% Check where are the points to be included
NOTRUE = 0;
WTRUE = [];

for i=1:length(worldPoints)
    if world_pts_struct_t8(i).Include(1) == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

worldPointsGlobal_test8 = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        worldPointsGlobal_test8(i,j) = world_pts_struct_t8(WTRUE(i)).Value(j);
    end
end


% Image points: select the ones that are not NaN

imagePoints_test8 = zeros(NOTRUE,2,1);

for i=1:NOTRUE
    imagePoints_test8(i,1,1) = img_struct(8).Pixels(WTRUE(i)).Values(1);
    imagePoints_test8(i,2,1) = img_struct(8).Pixels(WTRUE(i)).Values(2);
end

% Calculate camera extrinsics
% The extrinsics are a transformation from world coordinates to camera coordinates 
% that enables you to transform points from the world coordinate system to the camera coordinate system.
camExtrinsics = estimateExtrinsics(imagePoints_test8,worldPointsGlobal_test8,intrinsics);

% Inverse transformation of the above (can be also obtained by doing inv(camExtrinsics.A)) 
cameraPose = extr2pose(camExtrinsics);

% This translation is relative to the origin of the set of world points we are
% using, orign that we set on the first point of the set

xyz = cameraPose.Translation; % extract translation

% To check thi rotation
eul = rotm2eul(cameraPose.R); % default rotation is ZYX

figure;
hold on;
scatter3(worldPointsGlobal_test8(:,1), worldPointsGlobal_test8(:,2),zeros(size(worldPointsGlobal_test8,1),1),20,'g','filled')
scatter3(worldPointsGlobal_test8(5,1), worldPointsGlobal_test8(5,2),0,20,'r','filled')
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
axis equal

% Find absolute position of point 1 of the set w.r.t. origin
[p8_1x_abs,p8_1y_abs,p8_1z_abs] = latlon2local(grid_test8(5).Lat,grid_test8(5).Lon,grid_test8(1).Alt,origin);
p8_1_xyz_abs = [p8_1x_abs,p8_1y_abs,p8_1z_abs];

cam_pos_test8 = p8_1_xyz_abs + xyz; % absolute griund positon of the camera

figure()
subplot(1,2,1)
imshow([data_dir, 'test8.png'])
hold on;
plot(imagePoints(:,1,8),imagePoints(:,2,8),'go');
plot(params.ReprojectedPoints(:,1,8),params.ReprojectedPoints(:,2,8),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
drawnow;
title("Image file:" + strcat('test',num2str(WTRUE(8)),'.png'));
subplot(1,2,2)
hold on;
scatter3(worldPointsGlobal_test8(:,1), worldPointsGlobal_test8(:,2),zeros(size(worldPointsGlobal_test8,1),1),20,'g','filled')
scatter3(worldPointsGlobal_test8(5,1), worldPointsGlobal_test8(5,2),0,20,'r','filled')
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
plotCamera(AbsolutePose = extr2pose(params.PatternExtrinsics(8)),Color="blue");
axis equal


%% Kerb

% Check where are the points to be included
NOTRUE = 0;
WTRUE = [];

for i=1:length(world_pts_kerb)
    if world_pts_kerb(i).Include(1) == 'T'
        NOTRUE = NOTRUE + 1;
        WTRUE(NOTRUE) = i;
    end
end

% Create the worldPoints variable

worldPointsGlobal_kerb = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        worldPointsGlobal_kerb(i,j) = world_pts_kerb(WTRUE(i)).Value(j);
    end
end

% Create the image Points variable

imagePoints_kerb = zeros(NOTRUE,2,1);

for i=1:NOTRUE
    for j=1:2
        imagePoints_kerb(i,j) = pixel_values_k(WTRUE(i)).Values(j);
    end
end

camExtrinsics = estimateExtrinsics(imagePoints_kerb,worldPointsGlobal_kerb,intrinsics);

cameraPose = extr2pose(camExtrinsics);

xyz = cameraPose.Translation; % extract translation

eul = rotm2eul(cameraPose.R); % default rotation is ZYX

figure;
hold on;
scatter3(worldPointsGlobal_kerb(:,1), worldPointsGlobal_kerb(:,2),zeros(size(worldPointsGlobal_kerb,1),1),20,'g','filled')
scatter3(worldPointsGlobal_kerb(5,1), worldPointsGlobal_kerb(5,2),0,20,'r','filled')
scatter3(xyz(1),xyz(2),xyz(3),20,'b','filled')
axis equal

% Find absolute position of point 1 of the set w.r.t. origin
[pg_1x_abs,pg_1y_abs,pg_1z_abs] = latlon2local(kerb(5).Lat,kerb(5).Lon,kerb(5).Alt,origin);
pg_1_xyz_abs = [pg_1x_abs,pg_1y_abs,pg_1z_abs];

cam_pos_test_k = pg_1_xyz_abs + xyz; % absolute griund positon of the camera


%% Extract camera lat and long

[lat1,lon1,alt1] = local2latlon( cam_pos_test5(1), ...
    cam_pos_test5(2), ...
    cam_pos_test5(3), ...
    origin);


[lat2,lon2,alt2] = local2latlon( cam_pos_test8(1), ...
    cam_pos_test8(2), ...
    cam_pos_test8(3), ...
    origin);


[lat3,lon3,alt3] = local2latlon( cam_pos_test_k(1), ...
    cam_pos_test_k(2), ...
    cam_pos_test_k(3), ...
    origin);

%% export camera positions in kml file

kmlwritepoint('cp_30pts_3t.kml',[lat1,lat2,lat3],[lon1,lon2,lon3], ...
    [alt1,alt2,alt3],'Name',{'Cam pos test 5', 'Cam pos test 8', 'Cam pos test kerb'});