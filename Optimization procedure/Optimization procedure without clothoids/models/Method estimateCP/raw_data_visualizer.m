function raw_data_visualizer(struct_data,num_frame,string,dir_path,X_adj,Y_adj,XR,YR,XL,YL,XM,YM,Lp,Up,right_border,left_border,fov)
% possible implementations:
% plot image, plot borders, show telemetry guess, show map, show image+map
    closest_points = struct_data(num_frame).closest_pt;
     % plot upper and lower bound for the car projected in real world
    N = 100 ;
    t = linspace(-10,10,N) ;
    x_lb=struct_data(num_frame).x_telem_estim+Lp*cos(struct_data(num_frame).psi_estim)-t*sin(struct_data(num_frame).psi_estim) ;
    y_lb=struct_data(num_frame).y_telem_estim+Lp*sin(struct_data(num_frame).psi_estim)+t*cos(struct_data(num_frame).psi_estim) ;
    x_ub=struct_data(num_frame).x_telem_estim+Up*cos(struct_data(num_frame).psi_estim)-t*sin(struct_data(num_frame).psi_estim) ;
    y_ub=struct_data(num_frame).y_telem_estim+Up*sin(struct_data(num_frame).psi_estim)+t*cos(struct_data(num_frame).psi_estim) ;
    
    % plot left and right bounds for car projected in real world
    t = linspace(Lp,22,Up) ;
    x_rs=struct_data(num_frame).x_telem_estim+t*cos(struct_data(num_frame).psi_estim)-(left_border)*sin(struct_data(num_frame).psi_estim) ;
    y_rs=struct_data(num_frame).y_telem_estim+t*sin(struct_data(num_frame).psi_estim)+(left_border)*cos(struct_data(num_frame).psi_estim) ;
    x_ls=struct_data(num_frame).x_telem_estim+t*cos(struct_data(num_frame).psi_estim)-(right_border)*sin(struct_data(num_frame).psi_estim) ;
    y_ls=struct_data(num_frame).y_telem_estim+t*sin(struct_data(num_frame).psi_estim)+(right_border)*cos(struct_data(num_frame).psi_estim) ;
    
    % plot field of view
    N = 100 ;
    t = linspace(0,20,N) ;
    x_fovr=struct_data(num_frame).x_telem_estim+t*(cos(struct_data(num_frame).psi_estim)*cos(-fov))-t*(sin(struct_data(num_frame).psi_estim)*sin(-fov)) ;
    y_fovr=struct_data(num_frame).y_telem_estim+t*(sin(struct_data(num_frame).psi_estim)*cos(-fov))+t*(cos(struct_data(num_frame).psi_estim)*sin(-fov)) ;
    x_fovl=struct_data(num_frame).x_telem_estim+t*(cos(struct_data(num_frame).psi_estim)*cos(fov))-t*(sin(struct_data(num_frame).psi_estim)*sin(fov)) ;
    y_fovl=struct_data(num_frame).y_telem_estim+t*(sin(struct_data(num_frame).psi_estim)*cos(fov))+t*(cos(struct_data(num_frame).psi_estim)*sin(fov)) ;



    if string == "Plot image"
        imshow([dir_path,struct_data(num_frame).Filename])
    elseif string == "Plot borders"
        imshow(struct_data(num_frame).borders_binary_image)
    elseif string == "Show telemetry guess"
        fprintf('\n ----------------------- \n')
        fprintf('X position guess: %.3f \n', struct_data(num_frame).x_telem_estim)
        fprintf('Y position guess: %.3f \n', struct_data(num_frame).y_telem_estim)
        fprintf('Velocity guess: %.3f \n', struct_data(num_frame).v_telem_estim)
        fprintf('Yaw angle guess: %.3f \n', struct_data(num_frame).psi_estim)
        fprintf('Time step video: %.4f \n', struct_data(num_frame).Time_step_frame)
        fprintf('\n ----------------------- \n')
    elseif string == "Show map"
        figure()
        plot(X_adj,Y_adj)     % <- adjusted telemetry xy data
        hold on
        %plot(x_estim,y_estim) % <- estimated position frame by frame
        plot(XR,YR)           % <- right border based on mean clothoid
        plot(XL,YL)           % <- left border based on mean clothoid
        plot(XM,YM)           % <- mean clothoid
        scatter(struct_data(num_frame).x_telem_estim, ...
            struct_data(num_frame).y_telem_estim, 50, 'g','filled'); % <- single point estimated
        scatter(closest_points(1), closest_points(2), 50, 'r', 'filled'); % <- closest point
        plot(struct_data(num_frame).Rx_border_guess(:,1), ...
            struct_data(num_frame).Rx_border_guess(:,2),'go') % lower border estimation line of sight (approximately 4,67 m from the camera)
        plot(struct_data(num_frame).Lx_border_guess(:,1), ...
            struct_data(num_frame).Lx_border_guess(:,2),'go') % lower border estimation line of sight (approximately 4,67 m from the camera)
        plot(x_fovr,y_fovr,'b-')
        plot(x_fovl,y_fovl,'b-')
        plot(x_ub,y_ub,':')
        plot(x_lb,y_lb,':')
        plot(x_rs,y_rs,':')
        plot(x_ls,y_ls,':')
        legend('Car coordinates telemetry','estimation in specific zome',...
            'right border','left border','midline')
        axis equal
        xlim([struct_data(num_frame).x_telem_estim-15,struct_data(num_frame).x_telem_estim+15])
        ylim([struct_data(num_frame).y_telem_estim-5,struct_data(num_frame).y_telem_estim+25])
    elseif string == "Show image and map"
        subplot(1,2,1);
        plot(X_adj,Y_adj)     % <- adjusted telemetry xy data
        hold on
        %plot(x_estim,y_estim) % <- estimated position frame by frame
        plot(XR,YR)           % <- right border based on mean clothoid
        plot(XL,YL)           % <- left border based on mean clothoid
        plot(XM,YM)           % <- mean clothoid
        scatter(struct_data(num_frame).x_telem_estim, ...
            struct_data(num_frame).y_telem_estim, 50, 'g','filled'); % <- single point estimated
        scatter(closest_points(1), closest_points(2), 50, 'r', 'filled'); % <- closest point
        plot(struct_data(num_frame).Rx_border_guess(:,1), ...
            struct_data(num_frame).Rx_border_guess(:,2),'go') % lower border estimation line of sight (approximately 4,67 m from the camera)
        plot(struct_data(num_frame).Lx_border_guess(:,1), ...
            struct_data(num_frame).Lx_border_guess(:,2),'go') % lower border estimation line of sight (approximately 4,67 m from the camera)
        plot(x_fovr,y_fovr,'b-')
        plot(x_fovl,y_fovl,'b-')
        plot(x_ub,y_ub,':')
        plot(x_lb,y_lb,':')
        plot(x_rs,y_rs,':')
        plot(x_ls,y_ls,':')
        legend('Car coordinates telemetry','estimation in specific zome',...
            'right border','left border','midline')
        axis equal
        xlim([struct_data(num_frame).x_telem_estim-15,struct_data(num_frame).x_telem_estim+15])
        ylim([struct_data(num_frame).y_telem_estim-5,struct_data(num_frame).y_telem_estim+25])

        subplot(1,2,2);
        imshow([dir_path,struct_data(num_frame).Filename])
    end
end