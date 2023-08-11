# Folder content

In this folder you can find the Matlab files to be used for the optimization procedure in order to get all the parameters. The procedure should be the following:
1) Create a time array from the video: open video_prop_extraction.m and create a struct that saves the video timesteps; the video in not uploaded in the repository so please refer to the link attached in the "readme.md" file in the experimental-dataset folder.
2) Create a struct after image segmentation: open features_lanes.m and create the struct containing the binary borders detected
3) Create a raw data struct: open raw_data_creation.m and create a rawdata struct for first-guess results
4) Perform the optimization via min_solver.m

All the other Matlab files are Matlab functions that are called in the main scripts listed above.
