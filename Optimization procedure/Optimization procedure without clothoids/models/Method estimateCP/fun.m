function y = fun(x,Raw_data,CM,CL,z_k,theta_k,Lp,Up,circuit_data,frames,plotcallback,figure)


    cost_l_val = zeros(size(Raw_data,2),1);
    cost_r_val = zeros(size(Raw_data,2),1);
    
    
    for k=1:length(frames) %size(Raw_data,2)
%             fprintf('\n *************************** \n')
%             fprintf('  I am in the frame %d, number %d', frames(k),k)
%             fprintf('\n *************************** \n')
%         try
            
            % search points in world rf for current frame
            
            [w_r,w_l] = search_points(x,CM,Lp,Up,k,circuit_data, ...
                Raw_data(frames(k)).x_telem_estim, ...
                Raw_data(frames(k)).y_telem_estim);
    
            % Obtain variables for error calculation
            
            % def x_r: x_r is the x-coordinate of the point on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % world points in camera reference frame
%             w_r
%             w_l
            try
                xw_r = w_r(:,1);
            catch
                right_border1 = [];
                left_border1  = [];
                [ ~, ~, Ps_start, ~, ~, ~ ] = CM.closest_point(Raw_data(frames(k)).x_telem_estim, ...
                        Raw_data(frames(k)).y_telem_estim);
                [xr1,yr1] = CM.eval(Ps_start);
                Ps = Ps_start;
                reach = 0;
                cnt = 1;
                while ~reach
                    PWR = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_R,Ps);
                    [XR1, YR1] = CM.evaluate(Ps, - PWR);
                    right_border1 = [right_border1;[XR1,YR1]];

                    PWL = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_L,Ps);
                    [XL1, YL1] = CM.evaluate(Ps, + PWL);
                    left_border1 = [left_border1;[XL1,YL1]];

                    Ps = Ps + 1;
                    cnt = cnt + 1;
                    if cnt == 15
                        reach = 1;
                    end
                end
                
                right_border2 = [];
                left_border2  = [];
                Ps = Ps_start;
                reach = 0;
                cnt = 1;
                while ~reach
                    PWR = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_R,Ps);
                    [XR1, YR1] = CM.evaluate(Ps, - PWR);
                    right_border2 = [right_border2;[XR1,YR1]];

                    PWL = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_L,Ps);
                    [XL1, YL1] = CM.evaluate(Ps, + PWL);
                    left_border2 = [left_border2;[XL1,YL1]];

                    Ps = Ps + 1;
                    cnt = cnt + 1;
                    if cnt == 15
                        reach = 1;
                    end
                end

                N = 100 ;
                t = linspace(-10,10,N) ;
                x_lb=Raw_data(frames(k)).x_telem_estim+ ...
                    Lp*cos(CM.theta(Ps_start))-t*sin(CM.theta(Ps_start)) ;
                y_lb=Raw_data(frames(k)).y_telem_estim+ ...
                    Lp*sin(CM.theta(Ps_start))+t*cos(CM.theta(Ps_start)) ;
                x_ub=Raw_data(frames(k)).x_telem_estim+ ...
                    Up*cos(CM.theta(Ps_start))-t*sin(CM.theta(Ps_start)) ;
                y_ub=Raw_data(frames(k)).y_telem_estim+ ...
                    Up*sin(CM.theta(Ps_start))+t*cos(CM.theta(Ps_start)) ;
                
                x_lb2=(Raw_data(frames(k)).x_telem_estim)+ ...
                    (Lp+x(10))*cos(CM.theta(Ps_start))-t*sin(CM.theta(Ps_start)) ;
                y_lb2=(Raw_data(frames(k)).y_telem_estim)+ ...
                    (Lp+x(10))*sin(CM.theta(Ps_start))+t*cos(CM.theta(Ps_start)) ;
                x_ub2=(Raw_data(frames(k)).x_telem_estim)+ ...
                    (Up+x(11))*cos(CM.theta(Ps_start))-t*sin(CM.theta(Ps_start)) ;
                y_ub2=(Raw_data(frames(k)).y_telem_estim)+ ...
                    (Up+x(11))*sin(CM.theta(Ps_start))+t*cos(CM.theta(Ps_start)) ;


                figure()
                plot(Raw_data(frames(k)).x_telem_estim + x(12+3*(k-1)), ...
                    Raw_data(frames(k)).y_telem_estim + x(13+3*(k-1)),'r*')
                hold on
                plot(Raw_data(frames(k)).x_telem_estim , ...
                    Raw_data(frames(k)).y_telem_estim ,'ro')
                plot(right_border1(:,1),right_border1(:,2))
                plot(left_border1(:,1),left_border1(:,2))
                plot(right_border2(:,1),right_border2(:,2))
                plot(left_border2(:,1),left_border2(:,2))
                plot(x_lb,y_lb,':')
                plot(x_ub,y_ub,':')
                plot(x_lb2,y_lb2,':')
                plot(x_ub2,y_ub2,':')
                plot(xr1,yr1,'g*')
                axis equal
            end
        
            % def y_r: y_r is the y-coordinate of the point on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % world points in camera reference frame
            
            yw_r = w_r(:,2);
            
            % world point frame set -> project in cam ref frame -> x,y -> clothoid -> sample
        
              
        
            % the same for the left side
            
            xw_l = w_l(:,1);
            yw_l = w_l(:,2);
        
        
            % def px_r: px_r is the x-coordinate of the pixel on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % pixel points in camera reference frame
            
            px_r = Raw_data(frames(k)).pixel_list_rx(:,1);
        
            % def py_r: py_r is the y-coordinate of the pixel on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % pixel points in camera reference frame
            
            py_r = Raw_data(frames(k)).pixel_list_rx(:,2);
        
        
            % same for the left side
            px_l = Raw_data(frames(k)).pixel_list_lx(:,1);
            py_l = Raw_data(frames(k)).pixel_list_lx(:,2);
            
            
    
            [Cl,x_cam_l,y_cam_l,px_caml,py_caml] = cost_2(x,CL,px_l,py_l, xw_l,yw_l, Raw_data(frames(k)).psi_estim, ...
                Raw_data(frames(k)).x_telem_estim,Raw_data(frames(k)).y_telem_estim,z_k,theta_k,k,plotcallback);
    
            [Cr,x_cam_r,y_cam_r,px_camr,py_camr] = cost_2(x,CL,px_r,py_r, xw_r,yw_r, Raw_data(frames(k)).psi_estim, ...
                Raw_data(frames(k)).x_telem_estim,Raw_data(frames(k)).y_telem_estim,z_k,theta_k,k,plotcallback);
            
            cost_l_val(k) = Cl;
            cost_r_val(k) = Cr;

%             if k==1
            plotcallback(x_cam_l,y_cam_l,px_caml,py_caml, ...
                x_cam_r,y_cam_r,px_camr,py_camr,figure)
%             end
            

%         catch
%             continue
%         end
    end
%     fprintf('\n ---------------------------\n')
%     fprintf('\n ---------------------------\n')
%     fprintf('\n ---------------------------\n')
    y = sum(cost_r_val) + sum(cost_l_val);
%     fprintf('Y = %3.4f',y)
%     fprintf('\n ...........................\n')
%     fprintf('\n ...........................\n')
%     fprintf('\n ...........................\n')
    
end