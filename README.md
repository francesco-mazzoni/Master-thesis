# Master-thesis
Source codes used for my master thesis work. I wanted to achieve racing vehicle localization via video images recorded by its onboard camera on a track. Results are incomplete and I'm glad if someone collaborates to improve results. Drop me an emai at franz.mazzoni98@gmail.com for more information.

## 1st technique: calibrate the onboard camera via road patterns and estimate the extrinsic matrix in closed form solution
Use starting grid positions or chicane kerbs to get the intrinsic camera parameters and re-estimate the extrinsics to get the localization. This technique is defined in the files contained in the folders "Chicane" and "Starting grid". "Chicane" uses chicane kerbs as calibration patterns whereas "Starting grid" uses starting positions as patterns. In these two folders, "dataset_creation.m" can be used to create strut variables and save pixel locations. "param_estimate.m" defines the extrinsic and intrinsic parameters.

## 2nd technique: match the curves
Use partial telemetry data and video sources to perform a minimization. This provides the values of the intriniscs and the extrinsics for each frame.
The reference folder is "Optimization procedure" where two main methods for parameters identification are proposed.

## Future works
Due to various reasons, the methodology proposed doesn't work well. Anyway, there are some resources that I personally found in order to do some future work and improve the results. Resources are:
* Image sensor used for onboard camera broadcasting during some F1 seasons, https://www.resolveoptics.com/207-000-6-18mm-f2-8-31-miniature-zoom-lens/
* How can I get the intrinsic parameters from the data reported in the upper link? One possible way is to follow the table reported here https://en.wikipedia.org/wiki/Image_sensor_format#Table_of_sensor_formats_and_sizes
* This paper can be interesting: https://www.researchgate.net/publication/290535140_Camera_calibration_algorithm_with_multiple_regression_model_based_on_spline_transformation
* Other resources: https://it.mathworks.com/help/driving/ug/create-360-birds-eye-view-image.html
