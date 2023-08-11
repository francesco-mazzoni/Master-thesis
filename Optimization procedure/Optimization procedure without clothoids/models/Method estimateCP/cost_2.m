function [C,x_c,y_c,p_cx,p_cy] = cost_2(x,CL,px,py, x_w,y_w, psi,posx,posy,posz,theta,k,plotcallback)
    
% This cost function constructor aims at making two curves globally closer
% the one to each other. In order to do this, do not focus on points to be
% matched, but on their centroid. Create a virtual centre of mass for each
% curve of interest and try to make the centre of masses as close as
% possible

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


    % Create centroids

    centroid_pixels = [sum(p_cx)/length(p_cx) , ...
        sum(p_cy)/length(p_cy)];
    
    centroid_world = [sum(x_c)/length(x_c) , ...
        sum(y_c)/length(y_c)];



    
   C = (centroid_pixels(1)-centroid_world(1))^2+(centroid_pixels(2)-centroid_world(2))^2;

end