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

function myGoalState = goalstateTostruct(commonroad_goalstate)

    myGoalState=struct();
    myGoalState.position.type = []; % rectangle/circle/polygon/lanelet
    myGoalState.position.values = [];  % depends on shape
    myGoalState.orientation.intervalStart = NaN;
    myGoalState.orientation.intervalEnd = NaN;
    myGoalState.time.velocity = NaN;
    myGoalState.time.velocity= NaN;
    myGoalState.time.intervalStart = NaN;
    myGoalState.time.intervalEnd= NaN;


    numChild = size(commonroad_goalstate.Children,2);
    for k = 1:numChild
        switch commonroad_goalstate.Children(k).Name
            case 'position'
                
                %numPos = size(commonroad_goalstate.Children(k).Children,2);
                
                numPos = 1; % only take first, the other values are currently ignored.
                for i=1:numPos
                    % different ways to specify position
                    switch commonroad_goalstate.Children(k).Children(i).Name
                        
                        case 'rectangle'
                            myGoalState.position.type = 'rectangle';
                            length_R = NaN;
                            width_R = NaN;
                            % orientation and center may be omitted in XML if
                            % they equal zero
                            orientation_R = 0;
                            centerX_R = 0;
                            centerY_R = 0;
                            
                            numChild = size(commonroad_goalstate.Children(k).Children(i).Children,2);
                            for j=1:numChild
                                switch commonroad_goalstate.Children(k).Children(i).Children(j).Name
                                    case 'length'
                                        length_R = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                    case 'width'
                                        width_R = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                    case 'orientation'
                                        orientation_R = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                        
                                    case 'center'
                                        for m=1:2
                                            switch commonroad_goalstate.Children(k).Children(i).Children(j).Children(m).Name
                                                case 'x'
                                                    centerX_R = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children(m).Children.Data);
                                                case 'y'
                                                    centerY_R = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children(m).Children.Data);
                                            end
                                        end
                                end
                            end
                            myGoalState.position.values = [length_R,width_R,orientation_R,centerX_R,centerY_R ];
                        case 'circle'
                            myGoalState.position.type = 'circle';
                            radius_C = NaN;
                            centerX_C = 0;
                            centerY_C = 0;
                            
                            numChild = size(newObstacleNode.Children(k).Children(i).Children,2);
                            for j=1:numChild
                                switch newObstacleNode.Children(k).Children(i).Children(j).Name
                                    case 'radius'
                                        radius_C = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                    case 'center'
                                        % TODO: this if statement
                                        centerX_C = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                        centerY_C = str2double(commonroad_goalstate.Children(k).Children(i).Children(j).Children.Data);
                                        error('not implemented yet');
                                end
                            end
                            myGoalState.position.values = [radius_C,centerX_C,centerY_C ];
                            
                        case 'polygon'
                            myGoalState.position.type = 'polygon';
                            error('polygon shape not supported. please use minimum bounding rectangle');
                            
                        case 'lanelet'
                            myGoalState.position.type = 'lanelet';
                            % id
                            myGoalState.position.values = str2double(commonroad_goalstate.Children(k).Children(i).Attributes.Value);                            %                         myGoalState.position.values = lanelet_id ;
                            
                        otherwise
                            error(['unknown case : ', commonroad_goalstate.Children(j).Name]);
                    end
                end
            case 'time'
                for m=1:2
                    switch commonroad_goalstate.Children(k).Children(m).Name
                        case 'intervalStart'
                            myGoalState.time.intervalStart=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        case 'intervalEnd'
                            myGoalState.time.intervalEnd=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        otherwise
                            error(['unknown type: ', commonroad_goalstate.Children(k).Children(m).Name]);
                    end
                end

            case 'orientation'
                for m=1:2
                    switch commonroad_goalstate.Children(k).Children(m).Name
                        case 'intervalStart'
                            myGoalState.orientation.intervalStart=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        case 'intervalEnd'
                            myGoalState.orientation.intervalEnd=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        otherwise
                            error(['unknown type: ', commonroad_goalstate.Children(k).Children(m).Name]);
                    end
                end
            case 'velocity'
                for m=1:2
                    switch commonroad_goalstate.Children(k).Children(m).Name
                        case 'intervalStart'
                            myGoalState.velocity.intervalStart=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        case 'intervalEnd'
                            myGoalState.velocity.intervalEnd=  str2double(commonroad_goalstate.Children(k).Children(m).Children.Data);
                        otherwise
                            error(['unknown type: ', commonroad_goalstate.Children(k).Children(m).Name]);
                    end
                end
            otherwise
                error(['field name "', commonroad_goalstate.Children(k).Name,'" unknown']);
        end
    end
end