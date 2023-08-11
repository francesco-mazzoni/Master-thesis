function [c,ceq] = pos_and_or(x,rawdata,frames)

    c = [];
    ceq = [];
    for k=1:length(frames)-1
        xestim_idk0 = rawdata(frames(k)).x_telem_estim;
        xestim_idk1 = rawdata(frames(k+1)).x_telem_estim;
        yestim_idk0 = rawdata(frames(k)).y_telem_estim;
        yestim_idk1 = rawdata(frames(k+1)).y_telem_estim;
        
        psiestim    = rawdata(frames(k)).psi_estim;

        if frames(k+1)-frames(k) == 1 % successive frames
            ceq = [ceq;atan((yestim_idk1+x(13+3*(k-1))-yestim_idk0-x(13+3*k))/...
                (xestim_idk1+x(12+3*(k-1))-xestim_idk0+x(12+3*k))) ...
                - (psiestim+x(14+3*(k-1)))];
        end
    end
    
end