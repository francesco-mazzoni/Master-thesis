clc
clear all
close all

%% Extract dataset

data_dir = '../../experimental-data/Images for calibration/estimateCP method dataset/';
data_dir2 = '../../experimental-data/';
data_dir3 = '../../experimental-data/example_clothoids_Monza/circuit_data/';
data_dir4 = '../../experimental-data/Images for calibration/estimateCP method dataset/Test/';
dinfo = dir([data_dir, '*.mat']);

for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

dinfo = dir([data_dir2, '*.mat']);
for k = 1 : length(dinfo)
  thisfile = dinfo(k).name;
  load(fullfile(dinfo(k).folder,thisfile));
end

%% Extract telemetry data

telemetry = readtable([data_dir2,'Q3_ita_LEC_complete2.csv']);
t_telem = zeros(length(telemetry.Time),1);
for i=1:length(telemetry.Time)
    timestring = erase(cell2mat(telemetry.Time(i)),'0 days ');
    t_telem(i) = str2double(erase(string(duration(timestring,'InputFormat',...
        'hh:mm:ss.SSS','Format','s')),' sec'));
end
X = (telemetry.X - telemetry.X(1))/10;
Y = (telemetry.Y - telemetry.Y(1))/10;
velocity = telemetry.Speed;

%% Extract the right and left borders

circuit = readtable([data_dir3,'Monza_GE.txt']);

S = ClothoidSplineG2(); % Create a spline object G2
 
SM = S.buildP2(circuit.x_mid_line, circuit.y_mid_line );
SM.make_closed();

[XM, YM] = SM.evaluate(circuit.abscissa); % middle line
[XL, YL] = SM.evaluate(circuit.abscissa, + circuit.width_no_kerbs_L); % left line
[XR, YR] = SM.evaluate(circuit.abscissa, - circuit.width_no_kerbs_R); % right line

%% Plot telemetry and borders

adjust = [27.5,288];

figure()
plot(X-adjust(1),Y-adjust(2))
hold on
plot(XR,YR)
plot(XM,YM)
plot(XL,YL)
axis equal

X_adj = X-adjust(1);
Y_adj = Y-adjust(2);

%% load video timesteps

video_timesteps = load([data_dir2,'video.mat']).video.timesteps

%% Extract timesteps

fps = 50;
n_frame_start = 154;
n_frame_end = 4163; % = 85;

idx_video = find(video_timesteps(:)>= n_frame_start*(1/fps) & ...
    video_timesteps(:)<n_frame_end*(1/fps));
lower_bound = 10.6;
upper_bound = 13.3;
idx_test = [];

for i=1:length(idx_video)
    if video_timesteps(idx_video(i))-video_timesteps(idx_video(1))>lower_bound...
            && video_timesteps(idx_video(i))-video_timesteps(idx_video(1))<upper_bound
        idx_test = [idx_test,idx_video(i)];
    end
end

chicane_timestep = video_timesteps(idx_video);

chicane_timestep = chicane_timestep(chicane_timestep(:)>lower_bound & chicane_timestep(:)<=upper_bound);

%% State estimation

t_offset = 0.36;
x_estim = zeros(length(chicane_timestep),1);
y_estim = zeros(length(chicane_timestep),1);
v_estim = zeros(length(chicane_timestep),1);

for i=1:length(chicane_timestep)
    t_i = chicane_timestep(i) - t_offset;
    
    t_idk0 = max(t_telem(t_telem(:)<t_i));
    t_idk1 = min(t_telem(t_telem(:)>t_i));

    x_idk0 = X_adj((t_telem(:)==t_idk0));
    x_idk1 = X_adj((t_telem(:)==t_idk1));

    y_idk0 = Y_adj((t_telem(:)==t_idk0));
    y_idk1 = Y_adj((t_telem(:)==t_idk1));

    v_idk0 = velocity((t_telem(:)==t_idk0));
    v_idk1 = velocity((t_telem(:)==t_idk1));

    x_estim(i) = x_idk0 + (x_idk1 - x_idk0)/(t_idk1 - t_idk0) * (t_i - t_idk0);
    y_estim(i) = y_idk0 + (y_idk1 - y_idk0)/(t_idk1 - t_idk0) * (t_i - t_idk0);
    v_estim(i) = v_idk0 + (v_idk1 - v_idk0)/(t_idk1 - t_idk0) * (t_i - t_idk0);

end

%%

figure()
plot(X_adj,Y_adj)
hold on
plot(x_estim,y_estim,'r*')
axis equal

%% Find closest point to the mean clothoid and yaw angle

closest_pts = zeros(length(x_estim),2);
psi_estim   = zeros(length(x_estim),2);

for i=1:length(closest_pts)
    [ Px, Py, Ps, ~, ~, ~ ] = SM.closest_point(x_estim(i),y_estim(i));
    closest_pts(i,1) = Px;
    closest_pts(i,2) = Py;

    psi_estim(i) = SM.theta(Ps);
end

%% [Trial] Create a delimiter for points search

Lp = 4.67;
Up = 4.67 + 17.5;
right_border = 10;
left_border  = -10;
fov = 0.8;

% plot upper and lower bound for the car projected in real world
N = 100 ;
t = linspace(-10,10,N) ;
x_lb=x_estim(1)+Lp*cos(psi_estim(1))-t*sin(psi_estim(1)) ;
y_lb=y_estim(1)+Lp*sin(psi_estim(1))+t*cos(psi_estim(1)) ;
x_ub=x_estim(1)+Up*cos(psi_estim(1))-t*sin(psi_estim(1)) ;
y_ub=y_estim(1)+Up*sin(psi_estim(1))+t*cos(psi_estim(1)) ;

% plot left and right bounds for car projected in real world
t = linspace(Lp,22,Up) ;
x_rs=x_estim(1)+t*cos(psi_estim(1))-(left_border)*sin(psi_estim(1)) ;
y_rs=y_estim(1)+t*sin(psi_estim(1))+(left_border)*cos(psi_estim(1)) ;
x_ls=x_estim(1)+t*cos(psi_estim(1))-(right_border)*sin(psi_estim(1)) ;
y_ls=y_estim(1)+t*sin(psi_estim(1))+(right_border)*cos(psi_estim(1)) ;

% plot field of view
N = 100 ;
t = linspace(0,20,N) ;
x_fovr=x_estim(1)+t*(cos(psi_estim(1))*cos(-fov))-t*(sin(psi_estim(1))*sin(-fov)) ;
y_fovr=y_estim(1)+t*(sin(psi_estim(1))*cos(-fov))+t*(cos(psi_estim(1))*sin(-fov)) ;
x_fovl=x_estim(1)+t*(cos(psi_estim(1))*cos(fov))-t*(sin(psi_estim(1))*sin(fov)) ;
y_fovl=y_estim(1)+t*(sin(psi_estim(1))*cos(fov))+t*(cos(psi_estim(1))*sin(fov)) ;

%% [Trial] Plot zone of interest around the point

figure()
plot(X_adj,Y_adj)     % <- adjusted telemetry xy data
hold on
plot(XR,YR)           % <- right border based on mean clothoid
plot(XL,YL)           % <- left border based on mean clothoid
plot(XM,YM)           % <- mean clothoid
scatter(x_estim(1), y_estim(1), 50, 'g','filled'); % <- single point estimated
scatter(closest_pts(1,1), closest_pts(1,2), 50, 'r', 'filled'); % <- closest point
plot(x_fovr,y_fovr,'b-')
plot(x_fovl,y_fovl,'b-')
plot(x_ub,y_ub,':')
plot(x_lb,y_lb,':')
plot(x_rs,y_rs,':')
plot(x_ls,y_ls,':')
legend('Car coordinates telemetry','estimation in specific zome',...
'right border','left border','midline')
axis equal
xlim([x_estim(1)-15,x_estim(1)+15])
ylim([y_estim(1)-5,y_estim(1)+25])

%% [Mark the points (XR,YR), (XL,YL) inside the box
w_pts_of_interest(length(x_estim)) = struct();
%%
for k = 1:length(x_estim)
    start_idx = dsearchn([XR;YR]',[x_estim(k);y_estim(k)]')-3;
    exit_flag = 0;
    i = start_idx;
    right_finder = [];
    while ~exit_flag
        cond1 = (XR(i)-x_estim(k))*cos(psi_estim(k)) + (YR(i)-y_estim(k))*sin(psi_estim(k));
        cond2 = -(XR(i)-x_estim(k))*sin(psi_estim(k)) + (YR(i)-y_estim(k))*cos(psi_estim(k));
        
        
        if cond1>=Lp && cond1<=Up
            if cond2>=left_border && cond2<=right_border
                right_finder = [right_finder,i];
            end
        end
        
        if cond1>Up
            exit_flag = 1;
        end
        i = i + 1 ;
    end
    
    start_idx = dsearchn([XL;YL]',[x_estim(k);y_estim(k)]')-3;
    exit_flag = 0;
    i = start_idx;
    left_finder = [];
    while ~exit_flag
        cond1 = (XL(i)-x_estim(k))*cos(psi_estim(k)) + (YL(i)-y_estim(k))*sin(psi_estim(k));
        cond2 = -(XL(i)-x_estim(k))*sin(psi_estim(k)) + (YL(i)-y_estim(k))*cos(psi_estim(k));
        
        
        if cond1>=Lp && cond1<=Up
            if cond2>=left_border && cond2<=right_border
                left_finder = [left_finder,i];
            end
        end
        
        if cond1>Up
            exit_flag = 1;
        end
        i = i + 1 ;
    end
    w_pts_of_interest(k).right_border = [XR(right_finder);YR(right_finder)];
    w_pts_of_interest(k).left_border = [XL(left_finder);YL(left_finder)];
end

%%
figure()
plot(X_adj,Y_adj)     % <- adjusted telemetry xy data
hold on
plot(w_pts_of_interest(6).right_border(1,:)',...
    w_pts_of_interest(6).right_border(2,:)','go')           % <- right border based on mean clothoid
plot(w_pts_of_interest(6).left_border(1,:)',...
    w_pts_of_interest(6).left_border(2,:)','go');           % <- left border based on mean clothoid
plot(XM,YM)           % <- mean clothoid
scatter(x_estim(6), y_estim(6), 50, 'g','filled'); % <- single point estimated
scatter(closest_pts(6,1), closest_pts(6,2), 50, 'r', 'filled'); % <- closest point
plot(x_fovr,y_fovr,'b-')
plot(x_fovl,y_fovl,'b-')
plot(x_ub,y_ub,':')
plot(x_lb,y_lb,':')
plot(x_rs,y_rs,':')
plot(x_ls,y_ls,':')
legend('Car coordinates telemetry','estimation in specific zome',...
'right border','left border','midline')
axis equal
xlim([x_estim(6)-15,x_estim(6)+15])
ylim([y_estim(6)-5,y_estim(6)+25])

%%
rawdata(length(x_estim)) = struct();
z_estim = 0.97;
theta_estim = pi/20;

%% Raw data fill

for i=1:length(rawdata)

    M1 = [1,0,0,x_estim(i); ...
        0,1,0,y_estim(i); ...
        0,0,1,z_estim; ...
        0,0,0,1];
    M2 = [cos(psi_estim(i)),-sin(psi_estim(i)),0,0; ...
        sin(psi_estim(i)),cos(psi_estim(i)),0,0; ...
        0,0,1,0; ...
        0,0,0,1];
    M3 = [0,0,1,0; 0,1,0,0; -1,0,0,0; 0,0,0,1];
    M4 = [0,1,0,0; -1,0,0,0; 0,0,1,0; 0,0,0,1];
    M5 = [1,0,0,0; ...
        0,cos(theta_estim),sin(theta_estim),0; ...
        0,-sin(theta_estim),cos(theta_estim),0; ...
        0,0,0,1];

    rawdata(i).t_step    = chicane_timestep(i);
    rawdata(i).Filename     = strcat('test',num2str(i),'.png');
    rawdata(i).x_telem_estim   = x_estim(i);
    rawdata(i).y_telem_estim   = y_estim(i);
    rawdata(i).closest_pt = closest_pts(i,:);
    rawdata(i).psi_estim = psi_estim(i);
    rawdata(i).speed     = v_estim(i);
    rawdata(i).M_extr    = inv(M1*M2*M3*M4*M5);
    rawdata(i).Rx_border_guess = w_pts_of_interest(i).right_border';
    rawdata(i).Lx_border_guess = w_pts_of_interest(i).left_border';
    rawdata(i).binaryborders = masks_collector(i).Borders;
    rawdata(i).pixel_right = masks_collector(i).Right_border;
    rawdata(i).pixel_left  = masks_collector(i).Left_border;
    
end

%% Raw data visualizer
command_list = ["Plot image","Plot borders","Show telemetry guess", ...
    "Show map","Show image and map"];
raw_data_visualizer(rawdata,50,command_list(5),data_dir4, ...
    X_adj,Y_adj,XR,YR,XL,YL,XM,YM,Lp,Up,right_border,left_border,fov)

%% Raw data save

save([data_dir2,'rawdata_save_trial.mat'],"rawdata");