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

function myState = stateTostruct(commonroad_state)
    % only for 'exact' states

    myState.position = [NaN NaN]; % [x y]
    myState.orientation = NaN;
    myState.velocity = NaN;
    myState.time = NaN;
    myState.yawRate = NaN;
    myState.acceleration = NaN;
    myState.slipAngle = NaN;

    numChild = size(commonroad_state.Children,2);
    for j = 1:numChild
        switch commonroad_state.Children(j).Name
            case 'position'
                % different ways to specify position
                switch commonroad_state.Children(j).Children.Name
                    
                    case 'point'
                        for m=1:2
                            switch commonroad_state.Children(j).Children.Children(m).Name
                                case 'x'
                                    myState.position(1) = str2double(commonroad_state.Children(j).Children.Children(m).Children.Data);
                                case 'y'
                                    myState.position(2) = str2double(commonroad_state.Children(j).Children.Children(m).Children.Data);
                            end
                        end
                                            
                    otherwise
                        error(['unknown case : ', commonroad_state.Children(j).Name]);
                end
                
                

            case 'orientation'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.orientation = str2double(commonroad_state.Children(j).Children.Children.Data);
            case 'time'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.time = str2double(commonroad_state.Children(j).Children.Children.Data);
            case 'velocity'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.velocity = str2double(commonroad_state.Children(j).Children.Children.Data);
            case 'yawRate'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.yawRate = str2double(commonroad_state.Children(j).Children.Children.Data);

            case 'acceleration'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.acceleration = str2double(commonroad_state.Children(j).Children.Children.Data);

            case 'slipAngle'
                assert(strcmp(commonroad_state.Children(j).Children.Name,'exact'));
                myState.slipAngle = str2double(commonroad_state.Children(j).Children.Children.Data);

            otherwise
                error(['field name "', commonroad_state.Children(j).Name,'" unknown']);
        end
    end
end