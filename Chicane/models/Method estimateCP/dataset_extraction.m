clc
clear all
close all

%% Camera params

% Try to estimate camera parameters using the calibration function in
% Matlab; then compare the results with the one obtained

AUGMENT_FLAG_test = 1; % if one needs more points for validation images just activate this flag

%% Load all mat files
data_dir = '../../experimental-data/Images for calibration/estimateCP method dataset/Test/';
kml_data_dir = '../../experimental-data/kml keypoints/';

% If previously created, delete the .mat file containing the worldPoints
% and the pixels
dinfo = dir([data_dir, 'world*', 'pixel*']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  delete(fullfile(dinfo(k).folder,thisfile));
end

dinfo = dir([data_dir, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

FORGOT_FLAG = 0;

%% Define keypoints, image coordinates

if FORGOT_FLAG == 0
    close all
    imshow([data_dir,'test1.png'])
    imagePoints        = [test_px_1(9).Position(1),test_px_1(9).Position(2); ...
        test_px_1(8).Position(1),test_px_1(8).Position(2); ...
        test_px_1(7).Position(1),test_px_1(7).Position(2); ...
        test_px_1(6).Position(1),test_px_1(6).Position(2); ...
        test_px_1(5).Position(1),test_px_1(5).Position(2); ...
        test_px_1(4).Position(1),test_px_1(4).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_1(3).Position(1),test_px_1(3).Position(2); ...
        test_px_1(2).Position(1),test_px_1(2).Position(2); ...
        NaN,NaN; ...
        test_px_1(1).Position(1),test_px_1(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN];
    
    imagePoints(:,:,2) = [NaN,NaN; ...
        test_px_2(16).Position(1),test_px_2(16).Position(2); ...
        test_px_2(15).Position(1),test_px_2(15).Position(2); ...
        test_px_2(14).Position(1),test_px_2(14).Position(2); ...
        test_px_2(13).Position(1),test_px_2(13).Position(2); ...
        test_px_2(12).Position(1),test_px_2(12).Position(2); ...
        test_px_2(11).Position(1),test_px_2(11).Position(2); ...
        test_px_2(10).Position(1),test_px_2(10).Position(2); ...
        test_px_2(9).Position(1),test_px_2(9).Position(2); ...
        test_px_2(8).Position(1),test_px_2(8).Position(2); ...
        test_px_2(7).Position(1),test_px_2(7).Position(2); ...
        test_px_2(6).Position(1),test_px_2(6).Position(2); ...
        test_px_2(5).Position(1),test_px_2(5).Position(2); ...
        test_px_2(4).Position(1),test_px_2(4).Position(2); ...
        test_px_2(3).Position(1),test_px_2(3).Position(2); ...
        test_px_2(2).Position(1),test_px_2(2).Position(2); ...
        NaN,NaN; ...
        test_px_2(1).Position(1),test_px_2(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,3) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_3(8).Position(1),test_px_3(8).Position(2); ...
        test_px_3(7).Position(1),test_px_3(7).Position(2); ...
        test_px_3(6).Position(1),test_px_3(6).Position(2); ...
        test_px_3(5).Position(1),test_px_3(5).Position(2); ...
        test_px_3(4).Position(1),test_px_3(4).Position(2); ...
        test_px_3(3).Position(1),test_px_3(3).Position(2); ...
        test_px_3(2).Position(1),test_px_3(2).Position(2); ...
        test_px_3(1).Position(1),test_px_3(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,4) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_4(7).Position(1),test_px_4(7).Position(2); ...
        NaN,NaN; ...
        test_px_4(6).Position(1),test_px_4(6).Position(2); ...
        test_px_4(5).Position(1),test_px_4(5).Position(2); ...
        test_px_4(4).Position(1),test_px_4(4).Position(2); ...
        test_px_4(3).Position(1),test_px_4(3).Position(2); ...
        test_px_4(2).Position(1),test_px_4(2).Position(2); ...
        test_px_4(1).Position(1),test_px_4(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,5) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_5(8).Position(1),test_px_5(8).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_5(7).Position(1),test_px_5(7).Position(2); ...
        test_px_5(6).Position(1),test_px_5(6).Position(2); ...
        test_px_5(5).Position(1),test_px_5(5).Position(2); ...
        test_px_5(4).Position(1),test_px_5(4).Position(2); ...
        test_px_5(3).Position(1),test_px_5(3).Position(2); ...
        test_px_5(2).Position(1),test_px_5(2).Position(2); ...
        test_px_5(1).Position(1),test_px_5(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,6) = [
        NaN,NaN; ...
        test_px_6(10).Position(1),test_px_6(10).Position(2); ...
        test_px_6(9).Position(1),test_px_6(9).Position(2); ...
        test_px_6(8).Position(1),test_px_6(8).Position(2); ...
        test_px_6(7).Position(1),test_px_6(7).Position(2); ...
        test_px_6(6).Position(1),test_px_6(6).Position(2); ...
        test_px_6(5).Position(1),test_px_6(5).Position(2); ...
        test_px_6(4).Position(1),test_px_6(4).Position(2); ...
        test_px_6(3).Position(1),test_px_6(3).Position(2); ...
        test_px_6(2).Position(1),test_px_6(2).Position(2); ...
        NaN,NaN; ...
        test_px_6(1).Position(1),test_px_6(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,7) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_7(9).Position(1),test_px_7(9).Position(2); ...
        NaN,NaN; ...
        test_px_7(8).Position(1),test_px_7(8).Position(2); ...
        test_px_7(7).Position(1),test_px_7(7).Position(2); ...
        test_px_7(6).Position(1),test_px_7(6).Position(2); ...
        test_px_7(5).Position(1),test_px_7(5).Position(2); ...
        test_px_7(4).Position(1),test_px_7(4).Position(2); ...
        test_px_7(3).Position(1),test_px_7(3).Position(2); ...
        test_px_7(2).Position(1),test_px_7(2).Position(2); ...
        test_px_7(1).Position(1),test_px_7(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        ];
    
    imagePoints(:,:,8) = [
        test_px_8(12).Position(1),test_px_8(12).Position(2); ...
        test_px_8(11).Position(1),test_px_8(11).Position(2); ...
        test_px_8(10).Position(1),test_px_8(10).Position(2); ...
        test_px_8(9).Position(1),test_px_8(9).Position(2); ...
        test_px_8(8).Position(1),test_px_8(8).Position(2); ...
        test_px_8(7).Position(1),test_px_8(7).Position(2); ...
        test_px_8(6).Position(1),test_px_8(6).Position(2); ...
        test_px_8(5).Position(1),test_px_8(5).Position(2); ...
        test_px_8(4).Position(1),test_px_8(4).Position(2); ...
        test_px_8(3).Position(1),test_px_8(3).Position(2); ...
        test_px_8(2).Position(1),test_px_8(2).Position(2); ...
        test_px_8(1).Position(1),test_px_8(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...   
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        ];
    
    imagePoints(:,:,9) = [
        NaN,NaN; ...
        NaN,NaN; ...  
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...   
        test_px_9(8).Position(1),test_px_9(8).Position(2); ...
        test_px_9(7).Position(1),test_px_9(7).Position(2); ...
        test_px_9(6).Position(1),test_px_9(6).Position(2); ...
        test_px_9(5).Position(1),test_px_9(5).Position(2); ...
        test_px_9(4).Position(1),test_px_9(4).Position(2); ...
        test_px_9(3).Position(1),test_px_9(3).Position(2); ...
        NaN,NaN; ...
        test_px_9(2).Position(1),test_px_9(2).Position(2); ...
        NaN,NaN; ...
        test_px_9(1).Position(1),test_px_9(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...   
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...    
        ];
    
    imagePoints(:,:,10) = [
        NaN,NaN; ...
        NaN,NaN; ...  
        NaN,NaN; ...
        NaN,NaN; ...   
        NaN,NaN; ...   
        NaN,NaN; ...
        NaN,NaN; ...  
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        test_px_10(6).Position(1),test_px_10(6).Position(2); ...
        NaN,NaN; ...
        test_px_10(5).Position(1),test_px_10(5).Position(2); ...
        NaN,NaN; ...    
        test_px_10(4).Position(1),test_px_10(4).Position(2); ...
        test_px_10(3).Position(1),test_px_10(3).Position(2); ...
        test_px_10(2).Position(1),test_px_10(2).Position(2); ...
        test_px_10(1).Position(1),test_px_10(1).Position(2); ...   
        ];
    
    imagePoints(:,:,11) = [
        test_px_11(9).Position(1),test_px_11(9).Position(2); ...
        test_px_11(8).Position(1),test_px_11(8).Position(2); ...
        test_px_11(7).Position(1),test_px_11(7).Position(2); ...
        test_px_11(6).Position(1),test_px_11(6).Position(2); ...
        test_px_11(5).Position(1),test_px_11(5).Position(2); ...  
        test_px_11(4).Position(1),test_px_11(4).Position(2); ...
        NaN,NaN; ...  
        test_px_11(3).Position(1),test_px_11(3).Position(2); ...
        NaN,NaN; ...  
        test_px_11(2).Position(1),test_px_11(2).Position(2); ...
        NaN,NaN; ...  
        test_px_11(1).Position(1),test_px_11(1).Position(2); ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,12) = [
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_12(7).Position(1),test_px_12(7).Position(2); ...
        test_px_12(6).Position(1),test_px_12(6).Position(2); ...
        test_px_12(5).Position(1),test_px_12(5).Position(2); ...
        test_px_12(4).Position(1),test_px_12(4).Position(2); ...
        test_px_12(3).Position(1),test_px_12(3).Position(2); ...
        test_px_12(2).Position(1),test_px_12(2).Position(2); ...
        NaN,NaN; ...
        test_px_12(1).Position(1),test_px_12(1).Position(2); ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,13) = [
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_13(10).Position(1),test_px_13(10).Position(2); ...
        test_px_13(9).Position(1),test_px_13(9).Position(2); ...
        test_px_13(8).Position(1),test_px_13(8).Position(2); ...
        test_px_13(7).Position(1),test_px_13(7).Position(2); ...
        test_px_13(6).Position(1),test_px_13(6).Position(2); ...
        test_px_13(5).Position(1),test_px_13(5).Position(2); ...
        test_px_13(4).Position(1),test_px_13(4).Position(2); ...
        test_px_13(3).Position(1),test_px_13(3).Position(2); ...
        test_px_13(2).Position(1),test_px_13(2).Position(2); ...
        test_px_13(1).Position(1),test_px_13(1).Position(2); ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,14) = [
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        test_px_14(5).Position(1),test_px_14(5).Position(2); ...
        NaN,NaN; ... 
        test_px_14(4).Position(1),test_px_14(4).Position(2); ...
        test_px_14(3).Position(1),test_px_14(3).Position(2); ...
        test_px_14(2).Position(1),test_px_14(2).Position(2); ...
        test_px_14(1).Position(1),test_px_14(1).Position(2); ...
        ];
    
    imagePoints(:,:,15) = [
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_15(7).Position(1),test_px_15(7).Position(2); ...
        test_px_15(6).Position(1),test_px_15(6).Position(2); ...
        test_px_15(5).Position(1),test_px_15(5).Position(2); ...
        test_px_15(4).Position(1),test_px_15(4).Position(2); ...
        test_px_15(3).Position(1),test_px_15(3).Position(2); ...
        test_px_15(2).Position(1),test_px_15(2).Position(2); ...
        NaN,NaN; ...
        test_px_15(1).Position(1),test_px_15(1).Position(2); ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,16) = [
        NaN,NaN; ...
        test_px_16(11).Position(1),test_px_16(11).Position(2); ...
        test_px_16(10).Position(1),test_px_16(10).Position(2); ...
        test_px_16(9).Position(1),test_px_16(9).Position(2); ...
        test_px_16(8).Position(1),test_px_16(8).Position(2); ...
        test_px_16(7).Position(1),test_px_16(7).Position(2); ...
        test_px_16(6).Position(1),test_px_16(6).Position(2); ...
        test_px_16(5).Position(1),test_px_16(5).Position(2); ...  
        test_px_16(4).Position(1),test_px_16(4).Position(2); ...
        test_px_16(3).Position(1),test_px_16(3).Position(2); ...
        test_px_16(2).Position(1),test_px_16(2).Position(2); ...
        test_px_16(1).Position(1),test_px_16(1).Position(2); ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,17) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_17(12).Position(1),test_px_17(12).Position(2); ...
        test_px_17(11).Position(1),test_px_17(11).Position(2); ...
        test_px_17(10).Position(1),test_px_17(10).Position(2); ...
        test_px_17(9).Position(1),test_px_17(9).Position(2); ...
        test_px_17(8).Position(1),test_px_17(8).Position(2); ...
        test_px_17(7).Position(1),test_px_17(7).Position(2); ...
        test_px_17(6).Position(1),test_px_17(6).Position(2); ...
        test_px_17(5).Position(1),test_px_17(5).Position(2); ...  
        test_px_17(4).Position(1),test_px_17(4).Position(2); ...
        test_px_17(3).Position(1),test_px_17(3).Position(2); ...
        test_px_17(2).Position(1),test_px_17(2).Position(2); ...
        test_px_17(1).Position(1),test_px_17(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,18) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_18(11).Position(1),test_px_18(11).Position(2); ...
        test_px_18(10).Position(1),test_px_18(10).Position(2); ...
        test_px_18(9).Position(1),test_px_18(9).Position(2); ...
        test_px_18(8).Position(1),test_px_18(8).Position(2); ...
        test_px_18(7).Position(1),test_px_18(7).Position(2); ...
        test_px_18(6).Position(1),test_px_18(6).Position(2); ...
        test_px_18(5).Position(1),test_px_18(5).Position(2); ...  
        test_px_18(4).Position(1),test_px_18(4).Position(2); ...
        test_px_18(3).Position(1),test_px_18(3).Position(2); ...
        test_px_18(2).Position(1),test_px_18(2).Position(2); ...
        NaN,NaN; ...
        test_px_18(1).Position(1),test_px_18(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        ];
    
    imagePoints(:,:,19) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_19(9).Position(1),test_px_19(9).Position(2); ...
        test_px_19(8).Position(1),test_px_19(8).Position(2); ...
        test_px_19(7).Position(1),test_px_19(7).Position(2); ...
        test_px_19(6).Position(1),test_px_19(6).Position(2); ...
        test_px_19(5).Position(1),test_px_19(5).Position(2); ...  
        test_px_19(4).Position(1),test_px_19(4).Position(2); ...
        test_px_19(3).Position(1),test_px_19(3).Position(2); ...
        test_px_19(2).Position(1),test_px_19(2).Position(2); ...
        NaN,NaN; ...
        test_px_19(1).Position(1),test_px_19(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ... 
        ];
    
    imagePoints(:,:,20) = [
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_20(10).Position(1),test_px_20(10).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        test_px_20(9).Position(1),test_px_20(9).Position(2); ...
        test_px_20(8).Position(1),test_px_20(8).Position(2); ...
        test_px_20(7).Position(1),test_px_20(7).Position(2); ...
        test_px_20(6).Position(1),test_px_20(6).Position(2); ...  
        test_px_20(5).Position(1),test_px_20(5).Position(2); ...
        test_px_20(4).Position(1),test_px_20(4).Position(2); ...
        test_px_20(3).Position(1),test_px_20(3).Position(2); ...
        test_px_20(2).Position(1),test_px_20(2).Position(2); ...    
        test_px_20(1).Position(1),test_px_20(1).Position(2); ...
        ];
    
    imagePoints(:,:,21) = [
        test_px_21(12).Position(1),test_px_21(12).Position(2); ...
        test_px_21(11).Position(1),test_px_21(11).Position(2); ...
        test_px_21(10).Position(1),test_px_21(10).Position(2); ...
        test_px_21(9).Position(1),test_px_21(9).Position(2); ...
        test_px_21(8).Position(1),test_px_21(8).Position(2); ...
        test_px_21(7).Position(1),test_px_21(7).Position(2); ...
        test_px_21(6).Position(1),test_px_21(6).Position(2); ...
        test_px_21(5).Position(1),test_px_21(5).Position(2); ...
        test_px_21(4).Position(1),test_px_21(4).Position(2); ...
        test_px_21(3).Position(1),test_px_21(3).Position(2); ...
        test_px_21(2).Position(1),test_px_21(2).Position(2); ...
        test_px_21(1).Position(1),test_px_21(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
    
    imagePoints(:,:,22) = [
        test_px_22(14).Position(1),test_px_22(14).Position(2); ...
        test_px_22(13).Position(1),test_px_22(13).Position(2); ...
        test_px_22(12).Position(1),test_px_22(12).Position(2); ...
        test_px_22(11).Position(1),test_px_22(11).Position(2); ...
        test_px_22(10).Position(1),test_px_22(10).Position(2); ...
        test_px_22(9).Position(1),test_px_22(9).Position(2); ...
        test_px_22(8).Position(1),test_px_22(8).Position(2); ...
        test_px_22(7).Position(1),test_px_22(7).Position(2); ...
        test_px_22(6).Position(1),test_px_22(6).Position(2); ...
        test_px_22(5).Position(1),test_px_22(5).Position(2); ...
        test_px_22(4).Position(1),test_px_22(4).Position(2); ...
        test_px_22(3).Position(1),test_px_22(3).Position(2); ...
        NaN,NaN; ...
        test_px_22(2).Position(1),test_px_22(2).Position(2); ...
        NaN,NaN; ...
        test_px_22(1).Position(1),test_px_22(1).Position(2); ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        NaN,NaN; ...
        ];
end

%%

if FORGOT_FLAG == 1
    
    pixel_container = zeros(length(img_struct(1).Pixels),2,length(img_struct));
    
    for i=1:length(img_struct)
        for j=1:length(img_struct(i).Pixels)
            pixel_container(j,1,i) = img_struct(i).Pixels(j).Values(1);
            pixel_container(j,2,i) = img_struct(i).Pixels(j).Values(2);
        end
    end
end

%%

if FORGOT_FLAG == 0
    close all
    figure; 
    imshow([data_dir, 'test22.png'])
    hold on;
    plot(imagePoints(:,1,22),imagePoints(:,2,22),'go');
    text(imagePoints(:,1,22)+2,imagePoints(:,2,22)+5,cellstr(num2str((1:size(imagePoints,1))')))
    drawnow
end


%%

if FORGOT_FLAG == 1
    pixel_container = zeros(length(img_struct(1).Pixels),2,length(img_struct));
    
    for i=1:length(img_struct)
        for j=1:length(img_struct(i).Pixels)
            pixel_container(j,1,i) = img_struct(i).Pixels(j).Values(1);
            pixel_container(j,2,i) = img_struct(i).Pixels(j).Values(2);
        end
    end
end

%%

if FORGOT_FLAG == 1
    close all
    figure; 
    imshow([data_dir, 'test10.png'])
    hold on;
    plot(pixel_container(:,1,10),pixel_container(:,2,10),'go');
    text(pixel_container(:,1,10)+2,pixel_container(:,2,10)+5,cellstr(num2str((1:size(pixel_container,1))')))
    drawnow
end

%% Create image pixel struct

% Create a pixel storage struct for the following imagePoints() 3D matrix
% creation

% Define number of pixels and images
NOPX  = 28;
NOIMG = 22;

% Define a struct that for every image defines if it has to be included or
% not in the calibration and its pixels to be used
for i = 1:NOIMG

    % Define a struct that for every pixel defines if it has to be included or
    % not in the calibration
    for j = 1:NOPX
        pixel_values(j).Include = 'T';
        if FORGOT_FLAG == 0
            pixel_values(j).Values = imagePoints(j,:,i);
        else
            pixel_values(j).Values = pixel_container(j,:,i);
        end
    end

    img_struct(i).Include = 'T';
    img_struct(i).Pixels = pixel_values;
end

save([data_dir,'img_struct.mat'],'img_struct');


%% Geodetic coordinates extraction

O = kml2struct([kml_data_dir, 'Origine traguardo.kml']);
grid  = kml2struct([kml_data_dir, 'grid.kml']);
chicane = kml2struct([kml_data_dir, 'chicane 1 v2.kml']);
kerb = kml2struct([kml_data_dir, 'keypoints cordolo def.kml']);
origin = [O.Lat, O.Lon, O.Alt];

% origin = [45.629828413016030, 9.291562998765919, 190.8882878615457];

%% World points struct

ref = [chicane(24).Lat,chicane(24).Lon,chicane(24).Alt]; %24


for i=1:NOPX
    world_pts_struct(i).Include = 'T';

    [local1,local2]=latlon2local(chicane(i).Lat,chicane(i).Lon,ref(3),ref);
    world_pts_struct(i).Value(1) = local1;
    world_pts_struct(i).Value(2) = local2;
  
end

save([data_dir,'world_pts_struct.mat'],'world_pts_struct');

%% Worldpoints creation

NOTRUE = 0;

for i=1:NOPX
    if world_pts_struct(i).Include
        NOTRUE = NOTRUE + 1;
    end
end

for i=1:NOTRUE
    for j=1:2
        worldPoints(i,j) = world_pts_struct(i).Value(j);
    end
end

%%

figure()
plot( [worldPoints(:,1)],[worldPoints(:,2)],'*');
hold on
plot( [worldPoints(24,1)],[worldPoints(24,2)],'*');
axis equal
% plot(p5_1x,p5_1y,'go');
axis equal

%% Extrinsic parameters extraction

ref = [chicane(5).Lat,chicane(5).Lon,chicane(5).Alt];

notnanc = 0;

if FORGOT_FLAG == 0
    for i=1:length(imagePoints(:,:,2))
        if ~isnan(imagePoints(i,1,2))
            notnanc = notnanc + 1;
        end
    end
else
    for i=1:length(pixel_container(:,:,2))
        if ~isnan(pixel_container(i,1,2))
            notnanc = notnanc + 1;
        end
    end
end

WNOTNAN = zeros(1,notnanc);

notnanc = 0;

if FORGOT_FLAG == 0
    for i=1:length(imagePoints(:,:,2))
        if ~isnan(imagePoints(i,1,2))
            notnanc = notnanc + 1;
            WNOTNAN(notnanc) = i;
        end
    end
else
    for i=1:length(pixel_container(:,:,2))
        if ~isnan(pixel_container(i,1,2))
            notnanc = notnanc + 1;
            WNOTNAN(notnanc) = i;
        end
    end
end


for i=1:notnanc
    
    world_pts_struct_2(i).Include = 'T';

    [local1,local2]=latlon2local(chicane(WNOTNAN(i)).Lat,chicane(WNOTNAN(i)).Lon,ref(3),ref);
    world_pts_struct_2(i).Value(1) = local1;
    world_pts_struct_2(i).Value(2) = local2;
end

len_so_far_wps2 = length(world_pts_struct_2);
%%

% Augment the number of keypoints: consider the mean point between
% keypoints of the shorter segment inside the pattern, i.e. the mean
% position between points (referred to the original dataset) 3 and 4, 5 and
% 6 and so on

cnt = 2;
idx = 1;
if AUGMENT_FLAG_test == 1
    while cnt<len_so_far_wps2-1
        world_pts_struct_2(len_so_far_wps2+idx).Include = 'T';
        world_pts_struct_2(len_so_far_wps2+idx).Value(1) = (world_pts_struct_2(cnt).Value(1)+...
            world_pts_struct_2(cnt+1).Value(1))/2;
        world_pts_struct_2(len_so_far_wps2+idx).Value(2) = (world_pts_struct_2(cnt).Value(2)+...
            world_pts_struct_2(cnt+1).Value(2))/2;
        cnt = cnt + 2;
        idx = idx + 1;
    end
end

world_pts_struct_t2 = world_pts_struct_2;

save([data_dir,'world_pts_struct_t2.mat'],'world_pts_struct_t2');

%% Extrinsic parameters extraction

ref = [chicane(13).Lat,chicane(13).Lon,chicane(13).Alt];

notnanc = 0;

if FORGOT_FLAG == 0
    for i=1:length(imagePoints(:,:,13))
        if ~isnan(imagePoints(i,1,13))
            notnanc = notnanc + 1;
        end
    end
else
    for i=1:length(pixel_container(:,:,13))
        if ~isnan(pixel_container(i,1,13))
            notnanc = notnanc + 1;
        end
    end
end

WNOTNAN = zeros(1,notnanc);

notnanc = 0;

if FORGOT_FLAG == 0
    for i=1:length(imagePoints(:,:,13))
        if ~isnan(imagePoints(i,1,13))
            notnanc = notnanc + 1;
            WNOTNAN(notnanc) = i;
        end
    end
else
    for i=1:length(pixel_container(:,:,13))
        if ~isnan(pixel_container(i,1,13))
            notnanc = notnanc + 1;
            WNOTNAN(notnanc) = i;
        end
    end
end


for i=1:notnanc
    
    world_pts_struct_3(i).Include = 'T';

    [local1,local2]=latlon2local(chicane(WNOTNAN(i)).Lat,chicane(WNOTNAN(i)).Lon,ref(3),ref);
    world_pts_struct_3(i).Value(1) = local1;
    world_pts_struct_3(i).Value(2) = local2;
end

len_so_far_wps3 = length(world_pts_struct_3);

%%

% Augment the number of keypoints: consider the mean point between
% keypoints of the shorter segment inside the pattern, i.e. the mean
% position between points (referred to the original dataset) 3 and 4, 5 and
% 6 and so on

cnt = 1;
idx = 1;
if AUGMENT_FLAG_test == 1
    while cnt<len_so_far_wps3
        world_pts_struct_3(len_so_far_wps3+idx).Include = 'T';
        world_pts_struct_3(len_so_far_wps3+idx).Value(1) = (world_pts_struct_3(cnt).Value(1)+...
            world_pts_struct_3(cnt+1).Value(1))/2;
        world_pts_struct_3(len_so_far_wps3+idx).Value(2) = (world_pts_struct_3(cnt).Value(2)+...
            world_pts_struct_3(cnt+1).Value(2))/2;
        cnt = cnt + 2;
        idx = idx + 1;
    end
end

world_pts_struct_t3 = world_pts_struct_3;

save([data_dir,'world_pts_struct_t3.mat'],'world_pts_struct_t3');

%% Worldpoints creation

NOTRUE = 0;

for i=1:length(world_pts_struct_t3)
    if world_pts_struct_t3(i).Include == 'T'
        NOTRUE = NOTRUE + 1;
    end
end

worldPoints3 = zeros(NOTRUE,2);

for i=1:NOTRUE
    for j=1:2
        worldPoints3(i,j) = world_pts_struct_t3(i).Value(j);
    end
end

%%

figure()
plot( [worldPoints3(:,1)],[worldPoints3(:,2)],'*');
hold on
plot( [worldPoints3(5,1)],[worldPoints3(5,2)],'*');
axis equal
% plot(p5_1x,p5_1y,'go');
axis equal

%% Extra dataset (not done yet) based on extra elements (other kerbs and so on)

% close all
% imshow([data_dir,'test1_kerb.png'])
% 
% %% Kerb test
% 
% % Extract the data of a different element
% 
% % Image points
% 
% kerb_pixels = load('../../experimental-data/Images for calibration/estimateCP method dataset/3 and 2 starting positions/test1_px_k.mat').test1_px_k;
% 
% NOPX_k  = 12;
% 
% cnt = 0;
% 
% imagePoints_k = zeros(length(kerb_pixels),2,1);
% 
% for i=1:length(kerb_pixels)
%     imagePoints_k(i,1,1) = kerb_pixels(length(kerb_pixels)-cnt).Position(1);
%     imagePoints_k(i,2,1) = kerb_pixels(length(kerb_pixels)-cnt).Position(2);
%     cnt = cnt + 1;
% end
% 
% % Define a struct that for every pixel defines if it has to be included or
% % not in the calibration
% for j = 1:NOPX_k
%     if j < 9
%         pixel_values_k(j).Include = 'T';
%         pixel_values_k(j).Values = imagePoints_k(j,:,1);
%     elseif j == 9
%         pixel_values_k(j).Include = 'T';
%         pixel_values_k(j).Values(1) = (imagePoints_k(1,1,1)+imagePoints_k(2,1,1))/2;
%         pixel_values_k(j).Values(2) = (imagePoints_k(1,2,1)+imagePoints_k(2,2,1))/2;
%     elseif j == 10
%         pixel_values_k(j).Include = 'T';
%         pixel_values_k(j).Values(1) = (imagePoints_k(3,1,1)+imagePoints_k(4,1,1))/2;
%         pixel_values_k(j).Values(2) = (imagePoints_k(3,2,1)+imagePoints_k(4,2,1))/2;
%     elseif j == 11
%         pixel_values_k(j).Include = 'T';
%         pixel_values_k(j).Values(1) = (imagePoints_k(5,1,1)+imagePoints_k(6,1,1))/2;
%         pixel_values_k(j).Values(2) = (imagePoints_k(5,2,1)+imagePoints_k(6,2,1))/2;
%     elseif j == 12
%         pixel_values_k(j).Include = 'T';
%         pixel_values_k(j).Values(1) = (imagePoints_k(7,1,1)+imagePoints_k(8,1,1))/2;
%         pixel_values_k(j).Values(2) = (imagePoints_k(7,2,1)+imagePoints_k(8,2,1))/2;
%     end
% end
% 
% save([data_dir,'img_struct_k.mat'],'pixel_values_k');
% 
% 
% % World Points
% 
% ref = [kerb(1).Lat,kerb(1).Lon,kerb(1).Alt];
% 
% for i=1:12
%     world_pts_kerb(i).Include = 'T';
% 
%     if i < 9
%         [local1,local2]=latlon2local(kerb(i).Lat,kerb(i).Lon,ref(3),ref);
%         world_pts_kerb(i).Value(1) = local1;
%         world_pts_kerb(i).Value(2) = local2;
%     elseif i==9
%         world_pts_kerb(i).Value(1) = (world_pts_kerb(1).Value(1)+world_pts_kerb(2).Value(1))/2;
%         world_pts_kerb(i).Value(2) = (world_pts_kerb(1).Value(2)+world_pts_kerb(2).Value(2))/2;
%     elseif i==10
%         world_pts_kerb(i).Value(1) = (world_pts_kerb(3).Value(1)+world_pts_kerb(4).Value(1))/2;
%         world_pts_kerb(i).Value(2) = (world_pts_kerb(3).Value(2)+world_pts_kerb(4).Value(2))/2;
%     elseif i==11
%         world_pts_kerb(i).Value(1) = (world_pts_kerb(5).Value(1)+world_pts_kerb(6).Value(1))/2;
%         world_pts_kerb(i).Value(2) = (world_pts_kerb(5).Value(2)+world_pts_kerb(6).Value(2))/2;
%     elseif i==12
%         world_pts_kerb(i).Value(1) = (world_pts_kerb(7).Value(1)+world_pts_kerb(8).Value(1))/2;
%         world_pts_kerb(i).Value(2) = (world_pts_kerb(7).Value(2)+world_pts_kerb(8).Value(2))/2;
% 
%     end
% end
% 
% save([data_dir,'world_pts_struct_kerb.mat'],'world_pts_kerb');