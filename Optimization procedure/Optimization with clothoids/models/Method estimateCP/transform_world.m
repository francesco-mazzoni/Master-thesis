function [x_c,y_c] = transform_world(x,xw,yw,psi,posx,posy,posz,theta,k)
   
    
    x_c = world2camx(x);
    y_c = world2camy(x);
    z_c = world2camz(x);
    
    x_c = x_c/z_c;
    y_c = -y_c/z_c;

    function y = world2camx(x)

% (Y*cos(psi) - X*sin(psi)) - (B*cos(psi)) + (A*sin(psi))
        y = sin(psi + x(11+3*(k-1)))*xw ...
            -cos(psi + x(11+3*(k-1)))*yw ...
            -(posx + x(9+3*(k-1)))*sin(psi + x(11+3*(k-1)))+...
            (posy + x(10+3*(k-1)))*cos(psi + x(11+3*(k-1)));
    end

    
    function y = world2camy(x)

% (X*cos(psi)*sin(theta) + Y*sin(psi)*sin(theta) + Z*cos(theta)) - (A*cos(psi)*sin(theta)) - (B*sin(psi)*sin(theta))      
        y = -cos(psi + x(11+3*(k-1)))*sin(theta + x(7))*xw ...
            -sin(psi + x(11+3*(k-1)))*sin(theta + x(7))*yw ...
            +(posz + x(8))*cos(theta + x(7)) + ...
            (posx + x(9+3*(k-1)))*sin(theta + x(7))*cos(psi + x(11+3*(k-1)))...
            +(posy + x(10+3*(k-1)))*sin(theta + x(7))*sin(psi + x(11+3*(k-1)));
    end

    function y = world2camz(x)

% (X*cos(psi)*sin(theta) + Y*sin(psi)*sin(theta) + Z*cos(theta)) - (A*cos(psi)*sin(theta)) - (B*sin(psi)*sin(theta))      
        y = cos(psi + x(11+3*(k-1)))*cos(theta + x(7))*xw ...
            +sin(psi + x(11+3*(k-1)))*cos(theta + x(7))*yw ...
            +(posz + x(8))*sin(theta + x(7)) - ...
            (posx + x(9+3*(k-1)))*cos(theta + x(7))*cos(psi + x(11+3*(k-1)))...
            -(posy + x(10+3*(k-1)))*cos(theta + x(7))*sin(psi + x(11+3*(k-1)));  
    end
    
end