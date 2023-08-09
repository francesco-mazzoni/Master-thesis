function borders_bw = brd_identify(image_trial,y_th_up,y_th_low)

% image_trial = masks_collector(38).Mask_ROI;

for i=1:size(image_trial,1)
    for j=1:size(image_trial,2)
       if i<y_th_up || i>y_th_low
           image_trial(i,j) = 0;
       end
    end
end

% imshow(image_trial)
image_trial_lx = image_trial;
image_trial_rx = image_trial;

%% Consider the left side for testing

for i=1:size(image_trial,1)
    
    for j=1:size(image_trial,2)
       if j>=size(image_trial,2)/2
           image_trial_lx(i,j) = 0;
       end
    end
end

image_trial_lx_bc = image_trial_lx;

for i=1:size(image_trial,1)
    for j=1:size(image_trial,2)/2
        if i<y_th_up+0.3*(y_th_low-y_th_up)
            image_trial_lx_bc(i,j) = 0;
        end
    end
end
    

% imshow(image_trial_lx_bc)

cc = bwconncomp(image_trial_lx_bc,26);
    
lines = regionprops(cc,"PixelList","PixelIdxList");
lengths = zeros(1,length(lines));

for i=1:length(lines)
    lengths(i) = length(lines(i).PixelIdxList);
    lengths_ord = sort(lengths,'descend');
    if length(lines) > 1
        idx1 = find(lengths(:)==lengths_ord(1));
%         idx2 = find(lengths(:)==lengths_ord(2));
    else
        idx1 = 1;
    end
end

px1 = lines(idx1).PixelList;

cc1 = bwconncomp(image_trial_lx,26);
    
lines1 = regionprops(cc1,"PixelList","PixelIdxList");

for i=1:length(lines1)
    if ismember([px1(1,1), px1(1,2)],lines1(i).PixelList,'rows')
%         px_left = lines1(i).PixelList;
        line2save = i;
    end
end

% create new binary image
lx_border = image_trial_lx;
for i=1:length(lines1)
    if i~= line2save
        lx_border(lines1(i).PixelIdxList) = 0;
    end
end

% close all
% imshow(lx_border)
% hold on
% plot(px_left(:,1),px_left(:,2),'go')

%% Consider the right side for testing

for i=1:size(image_trial,1)
    for j=1:size(image_trial,2)
       if j<=size(image_trial,2)/2
           image_trial_rx(i,j) = 0;
       end
    end
end

imshow(image_trial_rx)

image_trial_rx_bc = image_trial_rx;

for i=1:size(image_trial,1)
    for j=size(image_trial,2)/2:size(image_trial,2)
        if i<y_th_up+0.3*(y_th_low-y_th_up)
            image_trial_rx_bc(i,j) = 0;
        end
    end
end

imshow(image_trial_rx_bc)

cc = bwconncomp(image_trial_rx_bc,26);
    
lines = regionprops(cc,"PixelList","PixelIdxList");
lengths = zeros(1,length(lines));

for i=1:length(lines)
    lengths(i) = length(lines(i).PixelIdxList);
    lengths_ord = sort(lengths,'descend');
    if length(lines) > 1
        idx1 = find(lengths(:)==lengths_ord(1));
%         idx2 = find(lengths(:)==lengths_ord(2));
    else
        idx1 = 1;
    end
end

px1 = lines(idx1).PixelList;

cc1 = bwconncomp(image_trial_rx,26);
    
lines1 = regionprops(cc1,"PixelList","PixelIdxList");

for i=1:length(lines1)
    if ismember([px1(1,1), px1(1,2)],lines1(i).PixelList,'rows')
%         px_right = lines1(i).PixelList;
        line2save = i;
    end
end

% create new binary image
rx_border = image_trial_rx;
for i=1:length(lines1)
    if i~= line2save
        rx_border(lines1(i).PixelIdxList) = 0;
    end
end

% close all
% imshow(rx_border)
% hold on
% plot(px_right(:,1),px_right(:,2),'go')

%% Return

borders_bw = or(rx_border,lx_border);
