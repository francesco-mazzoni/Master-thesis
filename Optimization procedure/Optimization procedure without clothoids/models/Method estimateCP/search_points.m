function [ret_right_lane,ret_left_lane] = search_points(x,CM,Lp,Up,k,circuit_data,x_est,y_est)
    right_lane = [];
    left_lane = [];
    left_border = -10;
    right_border = 10;
    [ ~, ~, Ps_start, ~, ~, ~ ] = CM.closest_point(x_est, y_est );
    
    Ps = Ps_start;
    exit_flag = 0;

    PWR = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_R,Ps_start);
    [XR1, YR1] = CM.evaluate(Ps_start, - PWR);

    while ~exit_flag

        PWR = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_R,Ps);

        [XR1, YR1] = CM.evaluate(Ps, - PWR);

        cond1 = (XR1-x_est )*cos(CM.theta(Ps_start))...
            + (YR1-y_est )*sin(CM.theta(Ps_start));

        cond2 = -(XR1-x_est)*sin(CM.theta(Ps_start))...
            + (YR1-y_est)*cos(CM.theta(Ps_start));
        
        if cond1>=(Lp+x(10)) && cond1<=(Up+x(11))
            if cond2>=left_border && cond2<=right_border
                right_lane = [right_lane;[XR1, YR1]];
            end
        end
        
        if cond1>(Up+x(11))
            exit_flag = 1;
        end
        Ps = Ps + 1 ;
    end
    
    Ps = Ps_start;
    exit_flag = 0;
    while ~exit_flag

        PWL = interp1(circuit_data.abscissa, circuit_data.width_no_kerbs_L,Ps); 

        [XL1, YL1] = CM.evaluate(Ps, + PWL);

        cond1 = (XL1-x_est )*cos(CM.theta(Ps_start))...
            + (YL1-y_est )*sin(CM.theta(Ps_start));

        cond2 = -(XL1-x_est)*sin(CM.theta(Ps_start))...
            + (YL1-y_est)*cos(CM.theta(Ps_start));
        
        
        if cond1>=(Lp+x(10)) && cond1<=(Up+x(11))
            if cond2>=left_border && cond2<=right_border
                left_lane = [left_lane;[XL1, YL1]];
            end
        end
        
        if cond1>(Up+x(11))
            exit_flag = 1;
        end
        Ps = Ps + 1 ;
    end
    ret_right_lane = right_lane;
    ret_left_lane  = left_lane;
end