function y = fun(x,Raw_data,CL,z_k,theta_k,frames)
    cost_l_val = zeros(size(Raw_data,2),1);
    cost_r_val = zeros(size(Raw_data,2),1);
    
    
    for k=1:length(frames) %size(Raw_data,2)
%         try
        
    
            % Obtain variables for error calculation
            
            % def x_r: x_r is the x-coordinate of the point on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % world points in camera reference frame
        
            xw_r = Raw_data(frames(k)).Rx_border_guess(:,1);
        
            % def y_r: y_r is the y-coordinate of the point on the clothoid spline
            % sampled at s=1 from the previous point considering the x-y projection of
            % world points in camera reference frame
            
            yw_r = Raw_data(frames(k)).Rx_border_guess(:,2);
            
            % world point frame set -> project in cam ref frame -> x,y -> clothoid -> sample
        
              
        
            % the same for the left side
            
            xw_l = Raw_data(frames(k)).Lx_border_guess(:,1);
            yw_l = Raw_data(frames(k)).Lx_border_guess(:,2);
        
        
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
            
%             fprintf('\n *********************** \n')
%             fprintf('I am in the frame %d, number %d', frames(k),k)
%             fprintf('\n *********************** \n')
    
            cost_l_val(k) =  cost(x,CL,px_l,py_l, xw_l,yw_l, Raw_data(frames(k)).psi_estim, ...
                Raw_data(frames(k)).x_telem_estim,Raw_data(frames(k)).y_telem_estim,z_k,theta_k,k);
    
            cost_r_val(k) = cost(x,CL,px_r,py_r, xw_r,yw_r, Raw_data(frames(k)).psi_estim, ...
                Raw_data(frames(k)).x_telem_estim,Raw_data(frames(k)).y_telem_estim,z_k,theta_k,k);
        
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