% data type for a 2D-point
classdef Point2D_RTree 
	properties (Access = public)
		x = double(0);
		y = double(0);
	end
	methods

        function obj = Point2D_RTree(x0,y0)

            if nargin == 2
                obj.x=x0;
                obj.y=y0;
            else
                % empty 
            end
        end
        
        function delete(obj)
        end
        function out=get_x(obj)
            out = obj.x;
        end
        function obj=set_x(obj,value)
            obj.x = value;
        end
        function out=get_y(obj)
            out = obj.y;
        end
        function obj=set_y(obj,value)
            obj.y = value;
        end
    end
end