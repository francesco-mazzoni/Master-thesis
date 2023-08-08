# Optimization using clothoids

This folder contains all the data and scripts for camera parameters identification following a custom optimization procedure.
The procedure consists in fitting projected curves with clothoids, sample them and minimize the square error between corresponding points.
Specifically, the projection occurs in the camera reference frame, meaning that the circuit borders are projected in this frame via roto-translation (extrinsic matrix)
and pixels are projected in the camera reference frame by inverting the intrinsic matrix. The projected points are fitted using G2-fitting splines and from them
one can sample the points every abscissa length s=1. Sampled points are used for the cost function construction by summing the square error of components.

DISCLAIMER: this procedure may be imperferct or not correct; the projection of pixels by inverting the intrinisc matrix produces rays, not points. Moreover, using clothoids
for curves results computationally expensive and prone to errors
