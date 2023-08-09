function [pc_x,pc_y] = transform_pixel(x,px,py)
    %    x(1) = f_x
    %    x(2) = f_y
    %    x(3) = o_x
    %    x(4) = o_y
    %    x(5) = k_1
    %    x(6) = k_2
    %    x(7) = d_theta
    %    x(8) = d_z
    %    x(9) = w
    
    pc_x = pixelbackx(x);
    pc_y = -pixelbacky(x);

    function y = pixelbackx(x)
        x_n =  (( px - x(3) ) / x(1));
        y_n =(( py - x(4) ) / x(2));
        y =  x_n*( 1 + x(5)*(x_n^2+y_n^2) + x(6)*(x_n^2+y_n^2)^2);
    end

    
    function y = pixelbacky(x)
        x_n = (( px - x(3) ) / x(1));
        y_n =  (( py - x(4) ) / x(2));
        y =  y_n*( 1 + x(5)*(x_n^2+y_n^2) + x(6)*(x_n^2+y_n^2)^2);
    end
end