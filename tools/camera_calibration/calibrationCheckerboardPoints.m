function calibrationCheckerboardPoints

    if exist('checkerboardPoints.mat','file')
        load checkerboardPoints
    else
        I=imread('/home/janis/Desktop/Image__2019-04-30__13-01-44.png');
        [imagePoints,boardSize] = detectCheckerboardPoints(I);
        save checkerboardPoints
    end
    
    imagePoints = reshape(imagePoints, boardSize(1)-1, boardSize(2)-1, 2);
    
    %imagePoints = flipud(imagePoints);
    imagePoints = fliplr(imagePoints);
    
    f = fopen('calibration_points.csv', 'w');
    fprintf(f,'world_x; world_y; image_x; image_y;\n');
    
    for ix = 1:size(imagePoints, 2)
        for iy = 1:size(imagePoints, 1)
            row = [(ix/10);(iy/10); squeeze(imagePoints(iy,ix,:))];
            fprintf(f,'%3.4f; ', row);
            fprintf(f,'\n');
        end
    end
    
    
    fclose(f);

    clf
    
    I = insertText(I,[imagePoints(1,end,1),imagePoints(1,end,2)],'x','FontSize',40);
    I = insertText(I,[imagePoints(end,1,1),imagePoints(end,1,2)],'y','FontSize',40);
    I = insertText(I,[imagePoints(1,1,1),imagePoints(1,1,2)],'origin','FontSize',40);
    imshow(I);
    hold on;
    plot(imagePoints(:,:,1),imagePoints(:,:,2),'rx');
    plot(imagePoints(1,1,1),imagePoints(1,1,2),'ro');
    

end

