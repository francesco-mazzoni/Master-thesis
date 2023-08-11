clear variables;
close all;
clc;

%% Load circuit data in a table
circuit_filepath = './circuit_data/Monza_GE.txt';
circuit_data = readtable(circuit_filepath);

%% Translate or rotate the circuit
% The first point of the circuit is on the center point of the starting
% line, and corresponds to the origin of the reference frame.
% Rotatranslation of the circuit must be done here, before building the
% clothoid list, by rototransalting

%% Build clothoid-list of the middle line
S = ClothoidSplineG2(); % Create a spline object G2
 
SM = S.buildP2(circuit_data.x_mid_line, circuit_data.y_mid_line );
SM.make_closed();

if 0
    figure;
    hold on
    SM.plot();
    axis equal
end


%% Compute cartesian coordinates of the two borders
% With ISO curvilinear reference frame positive direction of the lateral
% coordinate is to the left of abscissa direction

[XM, YM] = SM.evaluate(circuit_data.abscissa); % middle line
[XL, YL] = SM.evaluate(circuit_data.abscissa, + circuit_data.width_no_kerbs_L); % left line
[XR, YR] = SM.evaluate(circuit_data.abscissa, - circuit_data.width_no_kerbs_R); % right line

if 1
    figure;
    hold on
    plot(XM, YM, 'k');
    plot(XL, YL, 'b');
    plot(XR, YR, 'r');
    axis equal
end

%% Build clothoid-list of the two borders
% Building the clothoid list of the borders is not stricly necessary 
% SL = S.buildP2( XL, YL );
% SR = S.buildP2( XR, YR );
% 
% 
% if 0
%     figure;
%     hold on
%     SM.plot();
%     SL.plot();
%     SR.plot();
%     axis equal
% end

%% Example of finding closest point on the circuit

% Define an arbitrary point
Pxy = [60, 890];


% Find closest point on the middle line
[ Px, Py, Ps, Pt, iflag, Pdst ] = SM.closest_point(Pxy(1), Pxy(2));

%     > Evaluate the point at minimum distance of another point on the curve.
%     > `qx` and `qy` may be vectors so that the return values are vectors too.
%     >
%     > **Usage**
%     >
%     > \rst
%     > .. code-block:: matlab
%     >
%     >   [ x, y, s, t, iflag, dst ] = ref.closest_point( qx, qy );
%     >   [ x, y, s, t, iflag, dst ] = ref.closest_point( qx, qy, offs );
%     >   [ x, y, s, t, iflag, dst ] = ref.closest_point( qx, qy, offs, 'ISO' );
%     >   [ x, y, s, t, iflag, dst ] = ref.closest_point( qx, qy, offs, 'SAE' );
%     >
%     > \endrst
%     >
%     > **Optional Arguments**
%     >
%     > - offs: offset of the curve used in computation
%     > - 'ISO'/'SAE': use ISO or SAE orientation of the normal for the offset
%     >
%     > **Output**
%     >
%     > - `x`, `y`: Point at minimum distance from `(qx,qy)` on the curve.
%     > - `s`, `t`: Curvilinear coordinates of the point `(qx,qy)`.
%     > - `iflag`: `iflag < 0` some error in computation, iflag >0 is the numer of segment
%     >   containing the point at minimum distance.
%     > - `dst`: point curve distance.


% closest_point_ins_s_range can be used instead of closest_point to find
% the closest point in a prescribed abscissa range

figure;
hold on
plot(XM, YM, 'k');
plot(XL, YL, 'b');
plot(XR, YR, 'r');

scatter(Pxy(1), Pxy(2), 50, 'g','filled');
scatter(Px, Py, 50, 'r', 'filled');

axis equal
xlim([Pxy(1) - 50, Pxy(1) + 50]);
ylim([Pxy(2) - 50, Pxy(2) + 50]);

% Without creating the two clothoid list of the borders, the track width at
% the abscissa of the closest point can be found from the circuit table
% knowing Ps


PWL = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_L,Ps); % left width of the circuit from the closest point from Pxy
PWR = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_R,Ps); % right width of the circuit from the closest point from Pxy

[XL1, YL1] = SM.evaluate(Ps, + PWL);
[XR1, YR1] = SM.evaluate(Ps, - PWR);

figure;
hold on
plot(XM, YM, 'k');
plot(XL, YL, 'b');
plot(XR, YR, 'r');

scatter(Pxy(1), Pxy(2), 50, 'g','filled');
scatter(Px, Py, 50, 'r', 'filled');

plot(XL1,YL1, 'r*')
plot(XR1,YR1, 'g*')
axis equal

%% Evaluate the direction of the middle line at the abscissa value of the closest point

Ptheta = SM.theta(Ps); % return direction of clothoid list SM (the middle line in this case) at abscissa Ps






















