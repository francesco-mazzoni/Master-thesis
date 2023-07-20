# Master-thesis
Source codes used for my master thesis work. I wanted to achieve racing vehicle localization via video images recorded by its onboard camera on a track. Results are incomplete and I'm glad if someone collaborates to improve results. Drop me an emai at franz.mazzoni98@gmail.com for more information.

## 1st technique: calibrate the onboard camera via road patterns and estimate the extrinsic matrix in closed form solution
Use starting grid positions or chicane kerbs to get the intrinsic camera parameters and re-estimate the extrinsics to get the localization.

## 2nd technique: match the curves
Use partial telemetry data and video sources to perform a minimization. This provides the values of the intriniscs and the extrinsics for each frame
