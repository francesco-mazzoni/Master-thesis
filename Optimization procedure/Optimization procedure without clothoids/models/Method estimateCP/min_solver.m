clc
clear all
close all

%%

data_dir3 = '../../experimental-data/example_clothoids_Monza/circuit_data/';

%%

global CL

CL = ClothoidSplineG2();

%%

circuit = readtable([data_dir3,'Monza_GE.txt']);
 
CM = CL.buildP2(circuit.x_mid_line, circuit.y_mid_line );
CM.make_closed();

%% Height and Pitch guess

theta_k = 0;
z_k = 0.97;
Lp = 4.67;
Up = 4.67 + 17.50;
right_bound = 10;
left_bound = -10;

%% Load the data to use at each frame
data_dir = '../../experimental-data/';
Raw_data = load([data_dir,'Raw_data.mat']).Raw_data;


%% Cycle over the raw data to extract the global cost function

% Side note for unknown structure:
% -> total number of unknowns is 6 intrinsics + 1 pitch + 1 constant height + look ahead high and low (2) + 3*135 extrinsics (x,y,z,yaw)
%    = 11 + 405 = 414
% -> first eight places for constant parameters:
%    x(1) = f_x
%    x(2) = f_y
%    x(3) = o_x
%    x(4) = o_y
%    x(5) = k_1
%    x(6) = k_2
%    x(7) = d_theta
%    x(8) = d_z
%    x(9) = w
%    x(10)= d_Lp
%    x(11)= d_Up
% -> remaning 405 parameters:
%    x(12)=x(12+3*(k-1))   x(15)=x(12+3*(k-1))  x(18)=x(12+3*(k-1))  x(21)=x(12+3*(k-1))  ... = dx_k
%    x(13)=x(13+3*(k-1))   x(16)=x(13+3*(k-1))  x(19)=x(13+3*(k-1))  x(22)=x(13+3*(k-1))  ... = dy_k
%    x(14)=x(14+3*(k-1))   x(17)=x(14+3*(k-1))  x(20)=x(14+3*(k-1))  x(23)=x(14+3*(k-1))  ... = dpsi_k
%    k=1                   k=2                  k=3                  k=4                  ...

%% Parameters for minimization

% frames to be excluded: 1, 19, 20, 22-31, 50, 66, 71, 102, 118

frames2min = [2:18,21,51:65,67:70,72:101,103:117,119:135];

lb_ext = zeros(1,3*length(frames2min));
ub_ext = zeros(1,3*length(frames2min));

for i=1:length(frames2min)-1
    lb_ext(1+3*(i-1)) = -10;
    lb_ext(2+3*(i-1)) = -10;
    lb_ext(3+3*(i-1)) = -pi/5;

    ub_ext(1+3*(i-1)) = 10;
    ub_ext(2+3*(i-1)) = 10;
    ub_ext(3+3*(i-1)) = pi/5;

end


A   = [];
b   = [];
Aeq = [];
beq = [];
lb  = [-Inf,-Inf,0,0,-Inf,-Inf,-pi/10,-0.05,-Inf,-0.5,-0.5,lb_ext];
ub  = [Inf,Inf,size(Raw_data(1).borders_binary_image,2),...
    size(Raw_data(1).borders_binary_image,1),Inf,Inf,...
    pi/10,0.03,Inf,0.5,0.5,ub_ext];
%lb = [-Inf,-Inf,-Inf,-Inf,-Inf,-Inf,-Inf,-Inf,-Inf,0,0,-Inf*ones(1,3*length(frames2min))];
%ub = [Inf,Inf,Inf,Inf,Inf,Inf,Inf,Inf,Inf,0,0,Inf*ones(1,3*length(frames2min))];
% x0  = [1600,1600,size(Raw_data(1).borders_binary_image,2)/2, ...
%     size(Raw_data(1).borders_binary_image,1)/2,0,0,0,0,60,zeros(1,405)];
x0  = [1400,...  % f_x
    1400, ...    % f_y
    size(Raw_data(1).borders_binary_image,2)/2, ... % o_x
    size(Raw_data(1).borders_binary_image,1)/2, ... % o_y
    0, ... % k_1
    0, ... % k_2
    0, ... % theta
    0, ... % z
    1, ... % w
    0, ... % delta look-ahead distance
    0, ... % delta look-ahead distance 2
    zeros(1,3*length(frames2min)) ... % extrinsics
    ];


nonlcon = @(x)pos_and_or(x,Raw_data,frames2min);
% options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
% opts = optimoptions('fmincon','Algorithm','sqp',...
%     'MaxFunctionEvaluations', 1,'MaxIterations', 1,'MaxSQPIter',10,'Display','iter');

% opts = optimoptions('fmincon','Algorithm','interior-point','MaxFunctionEvaluations', inf,...
%     'MaxIterations', 2000,'PlotFcn',...
%     'optimplotfvalconstr','Display','iter');
opts_patts = optimoptions('patternsearch','Display','iter','PlotFcn','psplotfuncount','MaxFunctionEvaluations',300);
opts_parts = optimoptions('particleswarm','Display','iter','HybridFcn','patternsearch','PlotFcn','pswplotbestf');
opts = optimoptions('fmincon','Algorithm','interior-point', ...
    'PlotFcn','optimplotfvalconstr','Display','iter');


%% Preliminary debug | Data extraction

k=11;

right_world_border = Raw_data(frames2min(k)).Rx_border_guess;
left_world_border  = Raw_data(frames2min(k)).Lx_border_guess;

right_pixel_border = Raw_data(frames2min(k)).pixel_list_rx;
left_pixel_border = Raw_data(frames2min(k)).pixel_list_lx;

%% Preliminary debug | Original plots

figure()
plot(right_world_border(:,1),right_world_border(:,2))
hold on
plot(left_world_border(:,1),left_world_border(:,2))
axis equal
title('Right and left borders in xy global reference frame')

figure()
plot(right_pixel_border(:,1),-right_pixel_border(:,2))
hold on
plot(left_pixel_border(:,1),-left_pixel_border(:,2))
title('Right and left borders in xy image reference frame')
axis equal

%% Preliminary debug | Transformation world to camera

right_wrld2cam_border = zeros(size(right_world_border,1),size(right_world_border,2));
left_wrld2cam_border  = zeros(size(left_world_border,1),size(left_world_border,2));

for i=1:length(right_world_border)
    [x_camera_r_i,y_camera_r_i] = transform_world(x0,right_world_border(i,1), ...
        right_world_border(i,2),Raw_data(frames2min(k)).psi_estim, ...
        Raw_data(frames2min(k)).x_telem_estim,Raw_data(frames2min(k)).y_telem_estim, ...
        z_k,theta_k,k);

    right_wrld2cam_border(i,1) = x_camera_r_i;
    right_wrld2cam_border(i,2) = y_camera_r_i;
end

for i=1:length(left_world_border)
    [x_camera_l_i,y_camera_l_i] = transform_world(x0,left_world_border(i,1), ...
        left_world_border(i,2),Raw_data(frames2min(k)).psi_estim, ...
        Raw_data(frames2min(k)).x_telem_estim,Raw_data(frames2min(k)).y_telem_estim, ...
        z_k,theta_k,k);

    left_wrld2cam_border(i,1) = x_camera_l_i;
    left_wrld2cam_border(i,2) = y_camera_l_i;
end

%% Preliminary debug | Plot world to camera

figure()
plot(right_wrld2cam_border(:,1),right_wrld2cam_border(:,2))
hold on
plot(left_wrld2cam_border(:,1),left_wrld2cam_border(:,2))
axis equal
title('Right and left borders in xy camera reference frame (scaled)')

%% Preliminary debug | Transformation pixel to camera

right_px2cam_border = zeros(size(right_pixel_border,1),size(right_pixel_border,2));
left_px2cam_border  = zeros(size(left_pixel_border,1),size(left_pixel_border,2));

for i=1:length(right_pixel_border)
    [x_campx_r_i,y_campx_r_i] = transform_pixel(x0,right_pixel_border(i,1), ...
        right_pixel_border(i,2));

    right_px2cam_border(i,1) = x_campx_r_i;
    right_px2cam_border(i,2) = y_campx_r_i;
end

for i=1:length(left_pixel_border)
    [x_campx_l_i,y_campx_l_i] = transform_pixel(x0,left_pixel_border(i,1), ...
        left_pixel_border(i,2));

    left_px2cam_border(i,1) = x_campx_l_i;
    left_px2cam_border(i,2) = y_campx_l_i;
end

%% Preliminary debug | Plot pixel to camera

figure()
plot(right_px2cam_border(:,1),right_px2cam_border(:,2))
hold on
plot(left_px2cam_border(:,1),left_px2cam_border(:,2))
axis equal
title('Right and left borders in xy camera reference frame from pixel')

%%
f=figure;
%%

for k=1:length(frames2min)

right_world_border = Raw_data(frames2min(k)).Rx_border_guess;
left_world_border  = Raw_data(frames2min(k)).Lx_border_guess;

right_pixel_border = Raw_data(frames2min(k)).pixel_list_rx;
left_pixel_border = Raw_data(frames2min(k)).pixel_list_lx;

right_px2cam_border = zeros(size(right_pixel_border,1),size(right_pixel_border,2));
left_px2cam_border  = zeros(size(left_pixel_border,1),size(left_pixel_border,2));

for i=1:length(right_pixel_border)
    [x_campx_r_i,y_campx_r_i] = transform_pixel(x0,right_pixel_border(i,1), ...
        right_pixel_border(i,2));

    right_px2cam_border(i,1) = x_campx_r_i;
    right_px2cam_border(i,2) = y_campx_r_i;
end

for i=1:length(left_pixel_border)
    [x_campx_l_i,y_campx_l_i] = transform_pixel(x0,left_pixel_border(i,1), ...
        left_pixel_border(i,2));

    left_px2cam_border(i,1) = x_campx_l_i;
    left_px2cam_border(i,2) = y_campx_l_i;
end


plot(right_px2cam_border(:,1),right_px2cam_border(:,2))
hold on
plot(left_px2cam_border(:,1),left_px2cam_border(:,2))
axis equal
title(strcat('frame ',num2str(k)))
hold off
drawnow
pause(0.3)
end

%% Preliminary debug | Plot together

centroid_pixel_right = [sum(right_px2cam_border(:,1))/length(right_px2cam_border(:,1)),...
    sum(right_px2cam_border(:,2))/length(right_px2cam_border(:,2))];

centroid_pixel_left = [sum(left_px2cam_border(:,1))/length(left_px2cam_border(:,1)),...
    sum(left_px2cam_border(:,2))/length(left_px2cam_border(:,2))];

centroid_world_right = [sum(right_wrld2cam_border(:,1))/length(right_wrld2cam_border(:,1)),...
    sum(right_wrld2cam_border(:,2))/length(right_wrld2cam_border(:,2))];

centroid_world_left = [sum(left_wrld2cam_border(:,1))/length(left_wrld2cam_border(:,1)),...
    sum(left_wrld2cam_border(:,2))/length(left_wrld2cam_border(:,2))];

figure()
plot(right_px2cam_border(:,1),right_px2cam_border(:,2))
hold on
plot(left_px2cam_border(:,1),left_px2cam_border(:,2))
plot(right_wrld2cam_border(:,1),right_wrld2cam_border(:,2))
plot(left_wrld2cam_border(:,1),left_wrld2cam_border(:,2))
plot(centroid_pixel_left(1,1),centroid_pixel_left(1,2),'go')
plot(centroid_pixel_right(1,1),centroid_pixel_right(1,2),'g*')
plot(centroid_world_left(1,1),centroid_world_left(1,2),'ro')
plot(centroid_world_right(1,1),centroid_world_right(1,2),'r*')
axis equal
legend('px2cam right','px2cam left','world2cam right','world2cam left')
title('Camera frame overal projection')

%% Preliminary debug | Point selection for cost

cost(x0,CL,left_pixel_border(:,1),left_pixel_border(:,2), ...
    left_world_border(:,1),left_world_border(:,2),Raw_data(frames2min(k)).psi_estim,...
    Raw_data(frames2min(k)).x_telem_estim,Raw_data(frames2min(k)).y_telem_estim,...
    z_k,theta_k,k)

%% Preliminary debug | Possible algorithm

% 1) Find shortest curve: take four points for each curve and approximately
% compute the sum of lengths of the three segments

pt1 = [right_px2cam_border(1,1),right_px2cam_border(1,2)];
pt2 = [right_px2cam_border(floor( (1/3)*length(right_px2cam_border)),1),...
    right_px2cam_border(floor( (1/3)*length(right_px2cam_border)),2)];
pt3 = [right_px2cam_border(floor( (2/3)*length(right_px2cam_border)),1),...
    right_px2cam_border(floor( (2/3)*length(right_px2cam_border)),2)];
pt4 = [right_px2cam_border(length(right_px2cam_border),1),...
    right_px2cam_border(length(right_px2cam_border),2)];

len_px = sqrt((pt1(1)-pt2(1))^2+(pt1(2)-pt2(2))^2) + sqrt((pt2(1)-pt3(1))^2+(pt2(2)-pt3(2))^2) ...
    + sqrt((pt3(1)-pt4(1))^2+(pt3(2)-pt4(2))^2);

pt1 = [right_wrld2cam_border(1,1),right_wrld2cam_border(1,2)];
pt2 = [right_wrld2cam_border(floor( (1/3)*length(right_wrld2cam_border)),1),...
    right_wrld2cam_border(floor( (1/3)*length(right_wrld2cam_border)),2)];
pt3 = [right_wrld2cam_border(floor( (2/3)*length(right_wrld2cam_border)),1),...
    right_wrld2cam_border(floor( (2/3)*length(right_wrld2cam_border)),2)];
pt4 = [right_wrld2cam_border(length(right_wrld2cam_border),1),...
    right_wrld2cam_border(length(right_wrld2cam_border),2)];

len_wrd = sqrt((pt1(1)-pt2(1))^2+(pt1(2)-pt2(2))^2) + sqrt((pt2(1)-pt3(1))^2+(pt2(2)-pt3(2))^2) ...
    + sqrt((pt3(1)-pt4(1))^2+(pt3(2)-pt4(2))^2);

if len_px < len_wrd
    line_reference = right_px2cam_border;
    secondary_line = right_wrld2cam_border;
else
    line_reference = right_wrld2cam_border;
    secondary_line = right_px2cam_border;
end

%% Preliminary debug | Find the nearest point

idx = dsearchn(secondary_line,line_reference(1,:));
pt1s = [secondary_line(idx,1),secondary_line(idx,2)];

figure()
plot(right_px2cam_border(:,1),right_px2cam_border(:,2))
hold on
plot(right_wrld2cam_border(:,1),right_wrld2cam_border(:,2))
plot(pt1s(1),pt1s(2),'r*')
plot(line_reference(1,1),line_reference(1,2),'go')
axis equal

%% Preliminary debug | for every couple of the longest line find the matching pixel
% finish = 0;
% while ~finish
%     pt1s
% end

%% Minimization
% [sol0,fval0] = particleswarm(@(x) fun(x,Raw_data,CM,CL,z_k,theta_k,Lp,Up,circuit,frames2min) ,length(x0),lb,ub,opts_parts)
f=figure;
[sol0,fval0] = patternsearch(@(x) fun(x,Raw_data,CM,CL,z_k,theta_k,Lp,Up,circuit,frames2min,@plotcallback,f) ,x0,A,b,Aeq,beq,lb,ub,[],opts_patts)
%%

[sol,fval] = fmincon(@(x) fun(x,Raw_data,CM,CL,z_k,theta_k,Lp,Up,circuit,frames2min,@plotcallback,f) ,sol0,A,b,Aeq,beq,lb,ub,[],opts)

%% Plot position results
[XL, YL] = CM.evaluate(circuit.abscissa, + circuit.width_no_kerbs_L);
[XR, YR] = CM.evaluate(circuit.abscissa, - circuit.width_no_kerbs_R);
% telemetry = readtable([data_dir2,'Q3_ita_LEC_complete2.csv']);
% adjust = [27.5,288];
% X = (telemetry.X - telemetry.X(1))/10 - adjust(1);
% Y = (telemetry.Y - telemetry.Y(1))/10 - adjust(2);

%%
% example: frame 2 (index 1)


figure()
plot(XR,YR)
hold on
plot(XL,YL)
for k=1:length(frames2min)
    plot(Raw_data(frames2min(k)).x_telem_estim,Raw_data(frames2min(k)).y_telem_estim,'r*')
    plot((Raw_data(frames2min(k)).x_telem_estim+sol(12+3*(k-1))),(Raw_data(frames2min(k)).y_telem_estim+sol(13+3*(k-1))),'go')
end
title('Starting positions vs positions after estimation')
legend('Right border','Left border','Initial camera guess positions','Final camera positions')
axis equal

%%
psiout = zeros(length(frames2min),1);
psiout2 = zeros(length(frames2min),1);
for k=1:length(frames2min)
    psiout(k) = Raw_data(frames2min(k)).psi_estim;
    psiout2(k) = Raw_data(frames2min(k)).psi_estim+sol(14+3*(k-1));
end

figure()
plot(1:length(frames2min),psiout)
hold on
plot(1:length(frames2min),psiout2)
legend('psi before optim','psi after optim')
title('Initial yaw angle guess vs final yaw angle')

%% Pixel backproj

k = 10;

proj_rx = zeros(length(Raw_data(frames2min(k)).pixel_list_rx(:,1)),2);
proj_lx = zeros(length(Raw_data(frames2min(k)).pixel_list_lx(:,1)),2);

px1r = Raw_data(frames2min(k)).pixel_list_rx(:,1);
px2r = Raw_data(frames2min(k)).pixel_list_rx(:,2);

px1l = Raw_data(frames2min(k)).pixel_list_lx(:,1);
px2l = Raw_data(frames2min(k)).pixel_list_lx(:,2);

K = [sol(1),0,sol(3);0,sol(2),sol(4);0,0,1];
for i=1:length(px1r)

    [proj_rx(i,1),proj_rx(i,2)] = transform_pixel(sol,px1r(i),px2r(i));
    
end

for i=1:length(px1l)
    
    [proj_lx(i,1),proj_lx(i,2)] = transform_pixel(sol,px1l(i),px2l(i));
    
end

%% world points backproj

k = 10;

xwr = Raw_data(frames2min(k)).Rx_border_guess(:,1);
ywr = Raw_data(frames2min(k)).Rx_border_guess(:,2);

xwl = Raw_data(frames2min(k)).Lx_border_guess(:,1);
ywl = Raw_data(frames2min(k)).Lx_border_guess(:,2);

xwrp = zeros(length(Raw_data(frames2min(k)).Rx_border_guess(:,1)),1);
ywrp = zeros(length(Raw_data(frames2min(k)).Rx_border_guess(:,2)),1);

xwlp = zeros(length(Raw_data(frames2min(k)).Lx_border_guess(:,1)),1);
ywlp = zeros(length(Raw_data(frames2min(k)).Lx_border_guess(:,2)),1);

for i=1:length(xwrp)
    [xwrp(i),ywrp(i)] = transform_world(sol,xwr(i),ywr(i), ...
        Raw_data(frames2min(k)).psi_estim,Raw_data(frames2min(k)).x_telem_estim,...
        Raw_data(frames2min(k)).y_telem_estim,z_k,theta_k,k);
end

for i=1:length(ywlp)
    [xwlp(i),ywlp(i)] = transform_world(sol,xwl(i),ywl(i), ...
        Raw_data(frames2min(k)).psi_estim,Raw_data(frames2min(k)).x_telem_estim,...
        Raw_data(frames2min(k)).y_telem_estim,z_k,theta_k,k);
end

%%

figure()
plot(xwrp,ywrp)
hold on
plot(xwlp,ywlp)
plot(proj_lx(:,1),proj_lx(:,2))
plot(proj_rx(:,1),proj_rx(:,2))
axis equal
title('First tentative curve match in camera reference frame')
legend('World point projected, right side', 'World point projected, left side',...
    'Pixel projection, right side', 'Pixel projection, left side')


%%

% sample_px_l = proj_lx(1:50:length(proj_lx),:);
% sample_px_r = proj_rx(1:50:length(proj_rx),:);

sample_px_l = Raw_data(frames2min(k)).pixel_list_lx(1:20:length(...
    Raw_data(frames2min(k)).pixel_list_lx),:);
sample_px_r = Raw_data(frames2min(k)).pixel_list_rx(1:20:length(...
    Raw_data(frames2min(k)).pixel_list_rx),:);

lambdas_l = zeros(length(sample_px_l),1);
lambdas_r = zeros(length(sample_px_r),1);

xwbl = lambdas_l;
ywbl = lambdas_l;
xwbr = lambdas_r;
ywbr = lambdas_r;
%%
for i=1:length(sample_px_l)
    lambdas_l(i,1) = z_k / (sin(theta_k) + sample_px_l(i,2)*cos(theta_k));
    
    xwbl(i) = Raw_data(frames2min(k)).x_telem_estim + ...
        sample_px_l(i,1)*lambdas_l(i,1)*sin(Raw_data(frames2min(k)).psi_estim) +...
        lambdas_l(i,1)*cos(Raw_data(frames2min(k)).psi_estim)*cos(theta_k) -...
        sample_px_l(i,2)*lambdas_l(i,1)*cos(Raw_data(frames2min(k)).psi_estim)*sin(theta_k);
    
    ywbl(i) = Raw_data(frames2min(k)).y_telem_estim - ...
        sample_px_l(i,1)*lambdas_l(i,1)*cos(Raw_data(frames2min(k)).psi_estim) +...
        lambdas_l(i,1)*cos(theta_k)*sin(Raw_data(frames2min(k)).psi_estim) -...
        sample_px_l(i,2)*lambdas_l(i,1)*sin(Raw_data(frames2min(k)).psi_estim)*sin(theta_k);
end

for i=1:length(sample_px_r)
    lambdas_r(i,1) = z_k / (sin(theta_k) + sample_px_r(i,2)*cos(theta_k));
    
    xwbr(i) = Raw_data(frames2min(k)).x_telem_estim + ...
        sample_px_r(i,1)*lambdas_r(i,1)*sin(Raw_data(frames2min(k)).psi_estim) +...
        lambdas_r(i,1)*cos(Raw_data(frames2min(k)).psi_estim)*cos(theta_k) -...
        sample_px_r(i,2)*lambdas_r(i,1)*cos(Raw_data(frames2min(k)).psi_estim)*sin(theta_k);
    
    ywbr(i) = Raw_data(frames2min(k)).y_telem_estim - ...
        sample_px_r(i,1)*lambdas_r(i,1)*cos(Raw_data(frames2min(k)).psi_estim) +...
        lambdas_r(i,1)*cos(theta_k)*sin(Raw_data(frames2min(k)).psi_estim) -...
        sample_px_r(i,2)*lambdas_r(i,1)*sin(Raw_data(frames2min(k)).psi_estim)*sin(theta_k);
end

%%

figure;
plot(xwbr,-ywbr)
hold on
plot(xwbl,-ywbl)
axis equal