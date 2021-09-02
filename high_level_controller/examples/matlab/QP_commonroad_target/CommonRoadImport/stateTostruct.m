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