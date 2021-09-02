% class of Rectangle which is frequently used in the R-Tree

classdef Rectangle < handle

%  
% constructor : 
%               obj = Rectangle(in1,in2): overloaded
% destructor :
%               obj=delete(obj)
% functions  :
%               hasPoint = containsPoint(obj,evalPoint):
%                       checks if this rectangle contains the point
%                       evalPoint. returns 1 if yes, returns 0 if no
%                enlargedRect = enlargeForRectangle(obj,newRect):
%                       generates a new rectangle enlargedRect, which is
%                       the MBR for the union of this rectangle and newRect

    properties (Access = public)
        lle  ; % left lower edge
        rue ; % right upper edge
        area = double(0);
        empty = 0;
    end
    methods
        %% constructor method
        % Rectangle(lle_new,rue_new) new Rectangle with Points
        % Rectangle(Rectangle) copy constructor
        function obj = Rectangle(in1,in2) % left lower edge and right upper edge
            switch nargin
                case 0
                    % empty rectangle
                    obj.empty = 1;
                case 1
                    % Rectangle(Rectangle) copy constructor
                    obj.lle = in1.get_lle();
                    obj.rue = in1.get_rue();
                    deltaX = obj.rue.get_x() -  obj.lle().get_x();
                    deltaY = obj.rue.get_y() -  obj.lle().get_y();
                    obj.area= deltaX * deltaY;
                    
                    
                case 2
                    obj.lle=in1;
                    obj.rue=in2;
                    
                    deltaX = obj.rue.get_x() -  obj.lle().get_x();
                    deltaY = obj.rue.get_y() -  obj.lle().get_y();
                    
                    obj.area= deltaX * deltaY;
                    
                    
                    
                otherwise
                    error('not enough arguments in class rectangle');
                    
            end
        end
        
        %% class destructor
        function obj=delete(obj)
            if ~obj.empty
                obj.lle.delete();
                obj.rue.delete();
            end
        end
        
        %% geometry methods
        function hasPoint = containsPoint(obj,evalPoint)
            if obj.empty
                hasPoint = 0;
            else
                lb_x = (obj.get_lle().get_x() <= evalPoint.get_x());
                ub_x = (obj.get_rue().get_x() >= evalPoint.get_x());
                lb_y = (obj.get_lle().get_y() <= evalPoint.get_y());
                ub_y = (obj.get_rue().get_y() >= evalPoint.get_y());
                
                if  lb_x && ub_x && lb_y && ub_y
                    hasPoint = 1;
                else
                    hasPoint = 0;
                end
            end
        end
        function enlargedRect = enlargeForRectangle(obj,newRect)
            % function checking the necessary enlargement of this rectangle
            % to include the new rectangle
            
            if obj.empty
                enlargedRect = Rectangle(newRect.get_lle(),newRect.get_rue());
            else
                if obj.containsPoint(newRect.get_lle()) && obj.containsPoint(newRect.get_rue())
                    % rectangle already subset of the current rectangle
                    enlargedRect = obj;
                    return;
                else
                    new_lle_x = newRect.get_lle().get_x();
                    new_lle_y = newRect.get_lle().get_y();
                    new_rue_x = newRect.get_rue().get_x();
                    new_rue_y = newRect.get_rue().get_y();
                    
                    curr_lle_x =  obj.get_lle().get_x();
                    curr_lle_y =  obj.get_lle().get_y();
                    curr_rue_x =  obj.get_rue().get_x();
                    curr_rue_y =  obj.get_rue().get_y();
                    
                    
                    if  (new_lle_x < curr_lle_x )|| (new_lle_y < curr_lle_y )
                        new_lle = Point2D_RTree(min(new_lle_x,curr_lle_x),min(new_lle_y,curr_lle_y));
                    else
                        new_lle = obj.lle;
                    end
                    if  (new_rue_x > curr_rue_x )|| (new_rue_y > curr_rue_y )
                        new_rue = Point2D_RTree(max(new_rue_x,curr_rue_x),max(new_rue_y,curr_rue_y));
                        
                    else
                        new_rue = obj.rue;
                    end
                    enlargedRect = Rectangle(new_lle,new_rue);
                end
            end
        end
        %% get and setter
        function out=get_lle(obj)
            out = obj.lle;
        end
        function out=get_rue(obj)
            out = obj.rue;
        end
        %         function obj=set_lle(obj,value)
        %             obj.lle = value;
        %         end
        %         function obj=set_rue(obj,value)
        %             obj.rue = value;
        %         end
        
        function out_area=get_area(obj)
            
            out_area = obj.area;
        end
    end
    
    
    
    
end