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

%% this class should be used when creating an instance of a r-tree
classdef RTree < handle
    properties (Access = public)
        rootID = int32(0); % id of root node always 0
        RootNode;
        numLeafs=uint32(0); % counter
        numNodes = uint32(1); %  root node included

idList = MyList(); % list of ids
    end
    
    methods
        function obj=RTree()
            RectangleIn = Rectangle();
            newId= obj.rootID;      
            newContent=[];
            isRoot=1;
            RootNode = []; % dummy
            obj.RootNode=RNode(RectangleIn,newId,RootNode,newContent,isRoot);
            obj.RootNode.TreeObject=obj;
        end
        
        %% searches for Lanelets containing point xy_point = [xval, yval]
        % returns NodeList: Array of Lanelet ids
        function NodeArray = searchForLanelets(obj,xy_point)
            thePoint = Point2D_RTree(xy_point(1),xy_point(2));
            NodeArray = [];
            NodeArray = obj.RootNode.depthSearchLanelets(thePoint,NodeArray);

        end
        
        % returns list of leaf-RNodes which contain the Point 
        function NodeList = searchForRectangles(obj,thePoint)
            NodeList =MyList();
            NodeList = obj.RootNode.depthSearchRectangles(thePoint,NodeList);
            
        end
        
        % insert a newLeafNode of type RNode into the tree
        function obj= insert(obj,newLeafNode)
            
            obj.idList.appendElement(MyList(newLeafNode.get_Content().get_id()));

            newLeafNode.TreeObject = obj;
            LeafParent = obj.RootNode.chooseLeaf(newLeafNode);
            
            LeafParent.appendChild(newLeafNode);
            obj.numNodes = obj.numNodes +1;
            obj.numLeafs= obj.numLeafs + 1;
            
            LeafParent.adjustNode();
            
            Iterator = LeafParent;
            
            
            while Iterator.get_isRoot() ==0
                IteratorNew = Iterator.get_Parent();
                IteratorNew.adjustNode();
                if Iterator.get_toDelete()==1
                    Iterator.delete()
                end
                Iterator = IteratorNew;
            end
        end
        
        
        
        function nL = get_numLeafs(obj)
            
            nL = obj.numLeafs;
        end
        function l = get_idList(obj)
           l = obj.idList; 
        end
        
        % draw the RTree
        function drawTree(obj,fig)
            if nargin <1
                figure
            end
            hold on
            obj.RootNode.drawTree();
            hold off
        end
       
    end
    
    
end