function plotcallback(x_cam_l,y_cam_l,px_caml,py_caml, x_cam_r,y_cam_r,px_camr,py_camr,figure)
    
    figure;
    plot(x_cam_l,y_cam_l)
    hold on
    plot(x_cam_r,y_cam_r)
    plot(px_caml,py_caml)
    plot(px_camr,py_camr)
    plot(sum(x_cam_l)/length(x_cam_l),sum(y_cam_l)/length(y_cam_l),'go')
    plot(sum(x_cam_r)/length(x_cam_r),sum(y_cam_r)/length(y_cam_r),'ro')
    plot(sum(px_caml)/length(px_caml),sum(py_caml)/length(py_caml),'g*')
    plot(sum(px_camr)/length(px_camr),sum(py_camr)/length(py_camr),'r*')
%     plot(p_cx(1,[1:length(p_cx)]),p_cy(1,[1:length(p_cy)]),'r*')
%     plot(x_c([idx_start:idx_end]),y_c([idx_start:idx_end]),'bo')
%     plot(p_cx_match(1,:),p_cy_match(1,:),'go')

    axis equal
    legend('border left 2 cam','border right 2 cam','pixel left 2 cam','pixel right 2 cam')
    hold off
    drawnow
    pause(0.05)
end