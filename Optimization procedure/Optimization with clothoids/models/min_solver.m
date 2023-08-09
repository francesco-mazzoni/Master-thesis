clc
clear all
close all

%%

global CL

CL = ClothoidSplineG2();


%% Initialize cumulative error function


%% Height and Pitch guess

theta_k = pi/18;
z_k = 0.97;

%% Load the data to use at each frame
data_dir = '../../experimental-data/';
Raw_data = load([data_dir,'Raw_data.mat']).Raw_data;


%% Cycle over the raw data to extract the global cost function

% Side note for unknown structure:
% -> total number of unknowns is 6 intrinsics + 1 pitch + 1 constant height + 3*135 extrinsics (x,y,z,yaw)
%    = 9 + 405 = 414
% -> first eight places for constant parameters:
%    x(1) = f_x
%    x(2) = f_y
%    x(3) = o_x
%    x(4) = o_y
%    x(5) = k_1
%    x(6) = k_2
%    x(7) = d_theta
%    x(8) = d_z

% -> remaning 405 parameters:
%    x(9)=x(9+3*(k-1))   x(12)=x(10+3*(k-1))  x(16)=x(10+3*(k-1))  x(19)=x(10+3*(k-1))  ... = dx_k
%    x(10)=x(10+3*(k-1))   x(13)=x(11+3*(k-1))  x(17)=x(11+3*(k-1))  x(20)=x(11+3*(k-1))  ... = dy_k
%    x(11)=x(11+3*(k-1))   x(14)=x(12+3*(k-1))  x(18)=x(12+3*(k-1))  x(21)=x(12+3*(k-1))  ... = dpsi_k
%    k=1                   k=2                  k=3                  k=4                  ...

%% Parameters for minimization

frames2min = [3:10,55:80,125:130];

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
lb  = [-Inf,-Inf,-Inf,-Inf,-Inf,-Inf,0,-0.05,lb_ext];
ub  = [Inf,Inf,Inf,Inf,Inf,Inf,pi/18,0.03,ub_ext];
% x0  = [1600,1600,size(Raw_data(1).borders_binary_image,2)/2, ...
%     size(Raw_data(1).borders_binary_image,1)/2,0,0,0,0,60,zeros(1,405)];
x0  = [1800,1600,size(Raw_data(1).borders_binary_image,2)/2, ...
    size(Raw_data(1).borders_binary_image,1)/2,0,0,0.0001, ...
    0.0001,ones(1,3*length(frames2min))*0.0001];

% x01 = [sol(1),sol(2),sol(3),sol(4),sol(5),sol(6),sol(7), ...
%     sol(8),sol(9),ones(1,3*length(frames2min))*0.0001];
% 
% x02 = [sol1(1),sol1(2),sol1(3),sol1(4),sol1(5),sol1(6),sol1(7), ...
%     sol1(8),sol1(9),ones(1,3*length(frames2min))*0.0001];

nonlcon = [];
% options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
% opts = optimoptions('fmincon','Algorithm','sqp',...
%     'MaxFunctionEvaluations', 1,'MaxIterations', 1,'MaxSQPIter',10,'Display','iter');

% opts = optimoptions('fmincon','Algorithm','interior-point','MaxFunctionEvaluations', inf,...
%     'MaxIterations', 2000,'PlotFcn',...
%     'optimplotfvalconstr','Display','iter');
opts_ps = optimoptions('patternsearch','Display','iter','PlotFcn','psplotfuncount','MaxFunctionEvaluations',200);
opts = optimoptions('fmincon','Algorithm','interior-point', ...
    'MaxFunctionEvaluations', 3000,'PlotFcn','optimplotfvalconstr','Display','iter');
%% Minimization 1

[sol0,fval0] = patternsearch(@(x) fun(x,Raw_data,CL,z_k,theta_k,frames2min) ,x0,A,b,Aeq,beq,[],[],nonlcon,opts_ps)
%%
[sol0,fval0] = particleswarm(@(x) fun(x,Raw_data,CL,z_k,theta_k,frames2min) ,x0,A,b,Aeq,beq,[],[],nonlcon,[])

%% Minimization 2

[sol,fval] = fmincon(@(x) fun(x,Raw_data,CL,z_k,theta_k,frames2min) ,sol0,A,b,Aeq,beq,[],[],nonlcon,opts)


%%
k=25;
M = Raw_data(frames2min(k)).CW_rototransl;
r = Raw_data(frames2min(k)).Rx_border_guess;
l = Raw_data(frames2min(k)).Lx_border_guess;
pxr = Raw_data(frames2min(k)).pixel_list_rx;
pxl = Raw_data(frames2min(k)).pixel_list_lx;

xr = zeros(length(r),1);
yr = zeros(length(r),1);
zr = zeros(length(r),1);
a = xr;
b = yr;
c = xl;
d = yl;
for i=1:length(r)
    pr = M*[r(i,1);r(i,2);0;1];
    xr(i,1)=pr(1,1);
    yr(i,1)=pr(2,1);
    zr(i,1)=pr(3,1);
    [a(i,1),b(i,1)]=transform_world(x0,r(i,1),r(i,2),Raw_data(frames2min(k)).psi_estim,Raw_data(frames2min(k)).x_telem_estim,...
        Raw_data(frames2min(k)).y_telem_estim,z_k,theta_k,k);
    
end

xl = zeros(length(l),1);
yl = zeros(length(l),1);
zl = zeros(length(l),1);

for i=1:length(l)
    pr = M*[l(i,1);l(i,2);0;1];
    xl(i,1)=pr(1,1);
    yl(i,1)=pr(2,1);
    zl(i,1)=pr(3,1);
    [c(i,1),d(i,1)]=transform_world(x0,l(i,1),l(i,2),Raw_data(frames2min(k)).psi_estim,Raw_data(frames2min(k)).x_telem_estim,...
        Raw_data(frames2min(k)).y_telem_estim,z_k,theta_k,k);
end

figure;
plot(xr(:,1)./zr(:,1),-yr(:,1)./zr(:,1))
hold on
plot(xl(:,1)./zl(:,1),-yl(:,1)./zl(:,1))
plot(a(:,1),b(:,1),'go')
plot(c(:,1),d(:,1),'ro')
axis equal

K = [1100,0,540;0,1100,540;0,0,1];

pprx = zeros(length(pxr),1);
ppry = zeros(length(pxr),1);
e = pprx;
f = ppry;
for i=1:length(pxr)
    pppr = inv(K)*[pxr(i,1);pxr(i,2);1];
    pprx(i,1)=pppr(1,1);
    ppry(i,1)=pppr(2,1);
    [e(i,1),f(i,1)]=transform_pixel(x0,pxr(i,1),pxr(i,2));
end

pplx = zeros(length(pxl),1);
pply = zeros(length(pxl),1);
g = pplx;
h = pply;
for i=1:length(pxl)
    pppr = inv(K)*[pxl(i,1);pxl(i,2);1];
    pplx(i,1)=pppr(1,1);
    pply(i,1)=pppr(2,1);
    [g(i,1),h(i,1)]=transform_pixel(x0,pxl(i,1),pxl(i,2));
end

figure()
plot(pprx(:,1),-ppry(:,1))
hold on
plot(pplx(:,1),-pply(:,1))
plot(e,f,'go')
plot(g,h,'ro')
axis equal


figure()
plot(a,b)
hold on
plot(c,d)
plot(e,f,'go')
plot(g,h,'ro')
axis equal