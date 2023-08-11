function [C,x_c,y_c,p_cx,p_cy] = cost(x,CL,px,py, x_w,y_w, psi,posx,posy,posz,theta,k,plotcallback)
%     fprintf('Frame %d\n',k)
    p_cx = zeros(1,length(px));
    p_cy = zeros(1,length(py));
    ltotp = 0;
    for i=1:length(px)
        px_i = px(i);
        py_i = py(i);
        [p_cx_i,p_cy_i] = transform_pixel(x,px_i,py_i);
        p_cx(i) = p_cx_i;
        p_cy(i) = p_cy_i;
        if i>1
            ltotp = ltotp + sqrt((p_cx_i-p_cx(i-1))^2+(p_cy_i-p_cy(i-1))^2);
        end
    end
    
    x_c = zeros(1,length(x_w));
    y_c = zeros(1,length(y_w));
    ltot = 0;
    for i=1:length(x_w)
        xw_i = x_w(i);
        yw_i = y_w(i);
        [x_c_i,y_c_i] = transform_world(x,xw_i,yw_i,psi,posx,posy,posz,theta,k);
        x_c(i) = x_c_i;
        y_c(i) = y_c_i;
        if i>1
            ltot = ltot + sqrt((x_c_i-x_c(i-1))^2+(y_c_i-y_c(i-1))^2);
        end
    end
    
    % Matching points on pixel curve

    idx_start = dsearchn([x_c',y_c'],[p_cx(1,1),p_cy(1,1)]);
    idx_end   = dsearchn([x_c',y_c'],[p_cx(1,length(p_cx)),p_cy(1,length(p_cy))]);

    p_cx_match = zeros(1,idx_end-(idx_start-1)); %zeros(1,length(y_c));
    p_cy_match = zeros(1,idx_end-(idx_start-1)); %zeros(1,length(y_c));
    
    p_cx_match(1,1) = p_cx(1);
    p_cy_match(1,1) = p_cy(1);
    start = 2;
    adder = 0;
    found = 0;


%     figure()
%     plot(x_c,y_c)
%     hold on
%     plot(p_cx,p_cy)
% %     plot(p_cx(1,[1,pippo]),p_cy(1,[1,pippo]),'r*')
%     plot(x_c([idx_start:idx_end]),y_c([idx_start:idx_end]),'bo')
% %     plot(p_cx_match(1,:),p_cy_match(1,:),'go')
%     axis equal



    for i=1:length(p_cx_match)-1     %1:length(x_c)-1

        lp = sqrt((x_c(idx_start+1+adder)-x_c(idx_start+adder))^2+...
            (y_c(idx_start+1+adder)-y_c(idx_start+adder))^2);

%         fprintf('Length: %.4f\n',lp)
%         lp = (l*ltotp)/ltot;
%         lp = l;
        
        for j=start:length(p_cx)
            llp = sqrt((p_cx(j)-p_cx_match(i))^2+(p_cy(j)-p_cy_match(i))^2);
            if llp > lp-0.0005 && llp < lp+0.0005
%                 fprintf('eureka!!!!\n')
                foundidx = j;
                pippo = foundidx;
                start = j+1;
%                 fprintf('index of the pixel: %d\n',foundidx)
                found = 1;
            elseif llp > lp-0.001 && llp < lp+0.001 && ~found 
                foundidx = j;
                start = j+1;
                found = 1;
            elseif llp > lp-0.005 && llp < lp+0.005 && ~found  
                foundidx = j;
                start = j+1;
                found = 1;
            elseif llp > lp-0.01 && llp < lp+0.01 && ~found  
                foundidx = j;
                start = j+1;
                found = 1;
            elseif llp > lp-0.05 && llp < lp+0.05 && ~found 
%                 fprintf('IM here\n')
                foundidx = j;
                start = j+1;
                found = 1;
            elseif llp > lp-0.1 && llp < lp+0.1 && ~found
                foundidx = j;
                start = j+1;
                found = 1;
            elseif llp > lp-0.1005 && llp < lp+0.1005 && ~found
                foundidx = j;
                start = j+1;
                found = 1; 
            elseif llp > lp-0.101 && llp < lp+0.101 && ~found
                foundidx = j;
                start = j+1;
                found = 1; 
            elseif llp > lp-0.105 && llp < lp+0.105 && ~found
                foundidx = j;
                start = j+1;
                found = 1; 
            elseif llp > lp-0.11 && llp < lp+0.11 && ~found
                foundidx = j;
                start = j+1;
                found = 1; 
            elseif llp > lp-0.15 && llp < lp+0.15 && ~found
                foundidx = j;
                start = j+1;
                found = 1; 
%             else
%                 error('Frame %d has problems because lp is %.4f and llp is %.4f',k,lp,llp)
% %                 fprintf('im fuckin here\n')
%                 if start + floor(length(p_cx)/length(x_c)) < length(p_cx)
%                     foundidx = start + floor(length(p_cx)/length(x_c));
%                 else
%                     foundidx = length(p_cx);
%                 end
            end
        end
        

        try
%             i
            p_cx_match(1,i+1) = p_cx(foundidx);
            p_cy_match(1,i+1) = p_cy(foundidx);
            found = 0;
            adder = adder + 1;
        catch
            figure;
            plot(x_c,y_c)
            hold on
            plot(p_cx,p_cy)
            axis equal
        end
    end

   
    


%     p_cx = p_cx(1:floor(length(p_cx)/length(x_c))+1:length(p_cx));
%     p_cy = p_cy(1:floor(length(p_cy)/length(y_c))+1:length(p_cy)); 

%     cl_px = CL.buildP2(p_cx(1:floor(0.5*length(p_cx)):length(p_cx)),...
%         p_cy(1:floor(0.5*length(p_cy)):length(p_cy)));
%     
%     len = 0;
%     
%     p_cx_rsmpl = zeros(floor(cl_px.length())+1,1);
%     p_cy_rsmpl = zeros(floor(cl_px.length())+1,1);
%     while len < cl_px.length()
%         [p_cx_rsmpl_i,p_cy_rsmpl_i] = cl_px.eval(len);
%         len = len + 1;
%         p_cx_rsmpl(len) = p_cx_rsmpl_i;
%         p_cy_rsmpl(len) = p_cy_rsmpl_i;
%     end

%     cl_pt = CL.buildP2(x_c,y_c);
%     
%     len = 0;
%     x_c_rsmpl = zeros(floor(cl_pt.length())+1,1);
%     y_c_rsmpl = zeros(floor(cl_pt.length())+1,1);
%     while len < cl_pt.length()
%         [xc_rsmpl_i,yc_rsmpl_i] = cl_pt.eval(len);
%         len = len + 1;
%         x_c_rsmpl(len) = xc_rsmpl_i;
%         y_c_rsmpl(len) = yc_rsmpl_i;
%     end
    

    
    if length(x_c)==length(p_cx_match) && length(y_c)==length(p_cy_match)
        C = sum((x_c-p_cx_match).^2) + sum((y_c-p_cy_match).^2);
    else
        idx1 = min(length(x_c),length(p_cx_match));
        idx2 = min(length(y_c),length(p_cy_match));
        C = sum((x_c(1:idx1)-p_cx_match(1:idx1)).^2) + sum((y_c(1:idx2)-p_cy_match(1:idx2)).^2);
%         length(x_c)
%         length(p_cx)
%         error('Length mismatch')
    end


%     if length(x_c)<length(p_cx_rsmpl)
%         
%             
%             C = sum((x_c_rsmpl-p_cx_rsmpl(1:length(x_c_rsmpl))).^2) + ...
%                 sum((y_c_rsmpl-p_cy_rsmpl(1:length(y_c_rsmpl))).^2);
% 
% 
%     elseif length(x_c)>length(p_cx_rsmpl)
% 
%             C = sum((x_c_rsmpl(1:length(p_cx_rsmpl))-p_cx_rsmpl).^2) + ...
%                 sum((y_c_rsmpl(1:length(p_cy_rsmpl))-p_cy_rsmpl).^2);
%         
% 
%     else
%         
%             C = sum((x_c-p_cx_rsmpl).^2) + ...
%                 sum((y_c-p_cy_rsmpl).^2);
% 
%     end
end