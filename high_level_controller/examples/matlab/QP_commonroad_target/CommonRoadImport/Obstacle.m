classdef Obstacle < handle
    
    properties
        id;
        role;
        type;
        
        % shape.name = {rectangle, circle,polygon}
        % R: shape.values = [length_R,width_R,orientation_R,centerX_R,centerY_R ];
        % C: shape.values = [radius_C,centerX_C,centerY_C ];
        % P: shape.values = [length_R,width_R,orientation_R,center_R ];
        shape; 
        
        
        % State.position = [x y]
        % State.orientation = [angle]
        % State.velocity = [speed]
        % State.time = [timestep]
        initialState;
        
        % trajectory{numStates} as cell array of states 
        trajectory;
        
    end
    methods
        
        function obj=Obstacle(New_Obstacle)
            numAttributes = size(New_Obstacle.Attributes,1);
            for k=1:numAttributes
                if strcmp(New_Obstacle.Attributes.Name, 'id')
                    obj.id = New_Obstacle.Attributes.Value;
                    continue;
                end
            end
            numChildren = size(New_Obstacle.Children,2);
            for k=1:numChildren
                switch New_Obstacle.Children(k).Name
                    case 'role'
                        obj.role = New_Obstacle.Children(k).Children.Data;
                    case 'type'
                        obj.type = New_Obstacle.Children(k).Children.Data;
                    case 'shape'
                        obj.shape.name = New_Obstacle.Children(k).Children.Name;
                        switch obj.shape.name
                            case 'rectangle'
                                length_R = NaN;
                                width_R = NaN;
                                % orientation and center may be omitted in XML if
                                % they equal zero
                                orientation_R = 0;
                                centerX_R = 0;
                                centerY_R = 0;
                                
                                numChild = size(New_Obstacle.Children(k).Children.Children,2);
                                for j=1:numChild
                                    switch New_Obstacle.Children(k).Children.Children(j).Name
                                        case 'length'
                                            length_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children.Data);
                                        case 'width'
                                            width_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children.Data);
                                        case 'orientation'
                                            orientation_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children.Data);
                                            
                                        case 'center'
                                            
                                            for m=1:2
                                                switch New_Obstacle.Children(k).Children.Children(j).Children(m).Name
                                                    case 'x'
                                                        centerX_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children(m).Children.Data);
                                                    case 'y'
                                                        centerY_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children(m).Children.Data);
                                                end
                                            end
                                           
                                    end
                                end
                                obj.shape.values = [length_R,width_R,orientation_R,centerX_R,centerY_R ];
                            case 'circle'
                                radius_C = NaN;
                                centerX_C = 0;
                                centerY_C = 0;
                                
                                numChild = size(newObstacleNode.Children(k).Children.Children,2);
                                for j=1:numChild
                                    switch newObstacleNode.Children(k).Children.Children(j).Name
                                        case 'radius'
                                            radius_C = str2double(New_Obstacle.Children(k).Children.Children(j).Children.Data); 
                                        case 'center'
                                            for m=1:2
                                                switch New_Obstacle.Children(k).Children.Children(j).Children(m).Name
                                                    case 'x'
                                                        centerX_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children(m).Children.Data);
                                                    case 'y'
                                                        centerY_R = str2double(New_Obstacle.Children(k).Children.Children(j).Children(m).Children.Data);
                                                end
                                            end
                                    end
                                end
                                obj.shape.values = [radius_C,centerX_C,centerY_C ];
                           
                            case 'polygon'
                                error('polygon shape not supported. please use minimum bounding rectangle');
                            otherwise
                                error(['unknown shape: ', obj.shape.name ]);
                        end
                    
                    case 'initialState'
                        obj.initialState = stateTostruct(New_Obstacle.Children(k));
                        
                    case 'trajectory'
                        numStates = size(New_Obstacle.Children(k).Children,2);
                        obj.trajectory = cell(numStates,1);
                        for j = 1:numStates
                            obj.trajectory{j,1} = stateTostruct(New_Obstacle.Children(k).Children(j));
                        end
                        
                end
            
            end
        end
      
        
    end
end