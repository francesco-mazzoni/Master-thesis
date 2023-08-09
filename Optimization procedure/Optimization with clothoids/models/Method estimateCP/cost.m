function C = cost(x,CL,px,py, x_w,y_w, psi,posx,posy,posz,theta,k)
    p_cx = zeros(1,length(px));
    p_cy = zeros(1,length(py));
    for i=1:length(px)
        px_i = px(i);
        py_i = py(i);
        [p_cx_i,p_cy_i] = transform_pixel(x,px_i,py_i);
        p_cx(i) = p_cx_i;
        p_cy(i) = p_cy_i;
    end
    
    x_c = zeros(1,length(x_w));
    y_c = zeros(1,length(y_w));
    for i=1:length(x_w)
        xw_i = x_w(i);
        yw_i = y_w(i);
        [x_c_i,y_c_i] = transform_world(x,xw_i,yw_i,psi,posx,posy,posz,theta,k);
        x_c(i) = x_c_i;
        y_c(i) = y_c_i;
    end

  
%     p_cx
%     p_cy
%   

    % undersample
   
    try
        cl_px = CL.buildP2(p_cx,p_cy);
    catch
%         figure;
%         plot(p_cx,p_cy)
%         hold on
%         plot(p_cx(1:floor(0.15*length(p_cx)):length(p_cx)),...
%             p_cy(1:floor(0.15*length(p_cy)):length(p_cy)));
%         fprintf('scale factor: %f1.1', x(9))
        try
            cl_px = CL.buildP2(p_cx(1:10:length(p_cx)),p_cy(1:10:length(p_cy)));
            
        catch
%             figure;
%             plot(p_cx,p_cy)
%             hold on
%             plot(p_cx(1:floor(0.07*length(p_cx)):length(p_cx)),...
%                 p_cy(1:floor(0.07*length(p_cy)):length(p_cy)),'r*')
%             fprintf('scale factor: %f1.1', x(9))
            try
                cl_px = CL.buildP2(p_cx(1:20:length(p_cx)),p_cy(1:20:length(p_cy)));
            catch
                try
                    cl_px = CL.buildP2(p_cx(1:30:length(p_cx)),p_cy(1:30:length(p_cy)));
                catch
                    try
                        cl_px = CL.buildP2(p_cx(1:40:length(p_cx)),p_cy(1:40:length(p_cy)));
                    catch
                        try
                            cl_px = CL.buildP2(p_cx(1:50:length(p_cx)),p_cy(1:50:length(p_cy)));
                        catch
                            try
                                cl_px = CL.buildP2(p_cx(1:60:length(p_cx)),p_cy(1:60:length(p_cy)));
                            catch
                                cl_px = CL.buildP2(p_cx(1:70:length(p_cx)),p_cy(1:70:length(p_cy)));
                                figure()
                                plot(p_cx,p_cy)
                            end
                        end
                    end
                end
            end
        end
    end
    
    len = 0;
    
    p_cx_rsmpl = zeros(floor(cl_px.length())+1,1);
    p_cy_rsmpl = zeros(floor(cl_px.length())+1,1);
    while len < cl_px.length()
        [p_cx_rsmpl_i,p_cy_rsmpl_i] = cl_px.eval(len);
        len = len + 1;
        p_cx_rsmpl(len) = p_cx_rsmpl_i;
        p_cy_rsmpl(len) = p_cy_rsmpl_i;
    end
    
    try
        cl_pt = CL.buildP2(x_c,y_c);      
    catch
        figure;
        plot(x_c,y_c)
        hold on
        plot(x_c,y_c,'r*')
        try
            cl_pt = CL.buildP2(x_c(1:floor(0.05*length(x_c)):length(x_c)),...
                y_c(1:floor(0.05*length(y_c)):length(y_c)));
        catch
            cl_pt = CL.buildP2(x_c(1:floor(0.1*length(x_c)):length(x_c)),...
                y_c(1:floor(0.1*length(y_c)):length(y_c)));
        end
    end

    
    len = 0;
    x_c_rsmpl = zeros(floor(cl_pt.length())+1,1);
    y_c_rsmpl = zeros(floor(cl_pt.length())+1,1);
    while len < cl_pt.length()
        [xc_rsmpl_i,yc_rsmpl_i] = cl_pt.eval(len);
        len = len + 1;
        x_c_rsmpl(len) = xc_rsmpl_i;
        y_c_rsmpl(len) = yc_rsmpl_i;
    end
    
    

    if length(x_c_rsmpl)<length(p_cx_rsmpl)
        
            
            C = sum((x_c_rsmpl-p_cx_rsmpl(1:length(x_c_rsmpl))).^2) + ...
                sum((y_c_rsmpl-p_cy_rsmpl(1:length(y_c_rsmpl))).^2);


    elseif length(x_c_rsmpl)>length(p_cx_rsmpl)

            C = sum((x_c_rsmpl(1:length(p_cx_rsmpl))-p_cx_rsmpl).^2) + ...
                sum((y_c_rsmpl(1:length(p_cy_rsmpl))-p_cy_rsmpl).^2);
        

    else
        
            C = sum((x_c_rsmpl-p_cx_rsmpl).^2) + ...
                sum((y_c_rsmpl-p_cy_rsmpl).^2);

    end
end