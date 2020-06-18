% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

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

