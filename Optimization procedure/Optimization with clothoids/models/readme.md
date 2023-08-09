# Folder content

This folder contains all the Matlab scripts used for this methodology. The procedure to follow in order to get and use the proper dataset
and complete the optimization process is the following:

* Open video_prop_extraction.m and create a struct that saves the video timesteps.
* Create a struct after image segmentation: aprire features_lanes.m and create a sturct containing the borders in binary format
* Creation of a raw data struct: open raw_data_creation.m and create a dataset of first guess values to be used in the final optimization procedure
* Optimize using min_solver.m

Other Matlab files that are not mentioned can be regarded as auxiliary functions used in the main files mentioned.
