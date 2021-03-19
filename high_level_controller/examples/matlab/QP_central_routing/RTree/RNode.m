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

classdef RNode < handle
    
%     Constructor
%             RNode() : List head
%             RNode(RectangleIn,newId) :id given, internal node
%             RNode(0,0,0,isRoot) : root
%             RNode(RectangleIn,newId,newContent,isRoot,newPolygon) : leaf, with polygon
%             delete(obj) : Destructor, deletes Rectangle and Children Listhead as well
%     Function
%                         refreshRectangle(obj):   updates the rectangle of this node such that it covers the rectangle of the children
%             obj       = adjustNode(obj): adjustNodes if overflow happens, invokes split()
%             obj       = split(obj): splits the Node, uses quadratic effort, see Guttman paper
%             NodeList  = depthSearchLanelets(obj,thePoint,NodeArray):
%                           Searches the Subtree for lanelets containing the given Point
%                           and adds the id to the NodeArray
%             NodeList  = depthSearchRectangles(obj,thePoint,NodeList): Searches the Subtree for Rectangle leaf containing the given Point and appends it to the NodeList
%             LeafNode  = chooseLeaf(obj,newNode): chooses the Leaf in this Subtree to insert new Leaf
%                         drawNode(obj): draws the rectangle of this Node
%                         drawTree(obj): draws all the rectangle of the Rtree
%     ListFunctions for children list
%                   removeChild(obj,ChildIndex): removes a Child Node by its index in the list
%             obj = appendChild(obj,ChildNode): appends a ChildNode to the end of the list
%            
%     
%     Get
%             dl = get_toDelete(obj)
%             out = get_Rect(obj)
%             pg = get_Polygon(obj)
%             out = get_isLeaf(obj)
%             out = get_Content(obj)
%             out = get_id(obj)
%             out = get_isRoot(obj)
%             out = get_Parent(obj)
%             Children = get_Children(obj)
%             out = get_maxNum(obj)
%             out = get_minNum(obj)
%             out = get_numChild(obj)
%     Set
%              
%             obj = set_numChild(obj,value)
%             obj = set_Rect(obj,newRectangle)
%             obj = set_Polygon(obj,newPolygon)
%             obj =  set_id(obj,value)
%             obj = set_isLeaf(obj,value)
%             obj = set_isRoot(obj,value)
%             obj = set_Parent(obj,new_Parent)
%             obj = set_Children(obj,newChildren)
%             obj =  set_Container(obj,newContainer)

        
        
        
 
    
    
    
    
    
    
    
    
    properties (Access = public)
        Pred ;
        Succ ;
        hasSuccElement = 0;
        hasPredElement = 0;
        
        % max and min num of element per node
        maxNum = uint16(6);
        minNum = uint16(2);
        
        isLeaf = int8(0); % boolean
        isRoot = int8(0); % boolean
        hasContent = int8(0); % boolean
        Children  % List with children node
        numChild = uint16(0);
        Parent ; % parent node
        Rect ;
        Polygon; % as array of x,y points
        Content;
        id = int32(0);
        TreeObject;
        toDelete=int8(0);
    end
    
    methods (Access = public)
        function obj = RNode(RectangleIn,newId,RootNode,newContent,isRoot)
    
            
            switch nargin
                case 0
                    % empty initialization for list head of children list
                    obj.Rect = Rectangle(); 
                case 3
                    % creation of non-root nodes in r-tree
                    obj.Rect=RectangleIn;
                    obj.Children = RNode();
                    obj.id = newId;
                    obj.TreeObject = RootNode;

                case 5
                    % creation of root node
                    if isRoot == 1
                        
                        obj.Rect=RectangleIn;
                        obj.Children =  RNode();
                        obj.isRoot = 1;
                        obj.id =newId;
                        obj.isLeaf=1;
                        
                    else
                        obj.Rect=RectangleIn;
%                         obj.Children =  RNode();
                        obj.id = newId;
                        obj.Content = newContent;
                        obj.hasContent = 1;
                        obj.TreeObject = RootNode;
                    end
                otherwise
                    error('invalid number of argument in RNode()');
            end
            
        end
        
        function delete(obj)
            
        end
        
        function removeChild(obj,ChildIndex)
            obj.Children.removeByIndex(ChildIndex);
        end
        function obj = appendChild(obj,ChildNode)
            
            appendElement(obj.Children,ChildNode);
            obj.numChild=obj.numChild+1;
            ChildNode.set_Parent(obj);
            
        end
        
        % function which refreshes the MBR. this is necessary if one of the
        % child nodes has a new Rectangle
        function obj=refreshRectangle(obj)
            
            
            if obj.numChild>0
                newRect = Rectangle();
                Iterator = obj.Children;

                while Iterator.hasSucc()
                    Iterator = Iterator.get_Succ();
                    newRect = newRect.enlargeForRectangle(Iterator.get_Rect());
                end
               
%                obj.Rect.delete();
                obj.Rect= newRect;
               
            end
        end
        
        function obj=adjustNode(obj)
            if obj.numChild > obj.maxNum
                obj.split();
            else
                obj.refreshRectangle();
               
            end
            
        end
        
        function obj=split(obj)
            % Quadratic split algorithm, see Guttman apper
            numChildLeft = uint16(obj.numChild);
            if obj.isRoot
                obj.TreeObject.numNodes = obj.TreeObject.numNodes+1 ;   
                idA = obj.TreeObject.numNodes;
                obj.TreeObject.numNodes = obj.TreeObject.numNodes+1 ; 
                idB = obj.TreeObject.numNodes; 
            else
                obj.TreeObject.numNodes = obj.TreeObject.numNodes+1 ;   
                idA = obj.id;
                idB = obj.TreeObject.numNodes;     
            end
            
            % new Children Nodes
            Children_A = RNode(Rectangle(),idA,obj.TreeObject);
            Children_B = RNode(Rectangle(),idB,obj.TreeObject);
            Children_old = obj.Children;

            %% pick seed
            wastedAreaMax=0;
            SeedPair=[0 0];
            
            for k = 1:obj.numChild-1
                for j=k+1:obj.numChild
                    Rect_k=Children_old.getElement(k).get_Rect();
                    Rect_j=Children_old.getElement(j).get_Rect();
                    enlargedRect = Rect_k.enlargeForRectangle(Rect_j);
                    wastedArea = enlargedRect.get_area()- Rect_k.get_area() - Rect_j.get_area();
                    
                    if wastedArea > wastedAreaMax
                        SeedPair = [k j];
                        wastedAreaMax = wastedArea;
                    end
                    
                end
            end
            
            %% if they all contain each other
            if (SeedPair(1) == 0) && (SeedPair(2) ==0)
                Area = zeros(1,obj.numChild);
                for k=1:obj.numChild
                    Area(k)=Children_old.getElement(k).get_Rect().get_area();
                end
                [~,minIndex]=min(Area);
                [~,maxIndex]=max(Area);
                SeedPair= [minIndex(1),maxIndex(1)];
            end
            
            % append seeds to new children node
            FirstNodeA = Children_old.getElement(SeedPair(1));
            FirstNodeB = Children_old.getElement(SeedPair(2));
            FirstNodeA.removeThisElement();
            FirstNodeB.removeThisElement();
            
            Children_A.appendChild(FirstNodeA);
            Children_B.appendChild(FirstNodeB);
            numChildLeft = numChildLeft-2;
            GroupA_Rect = Children_A.get_Children().getElement(1).get_Rect();
            GroupB_Rect = Children_B.get_Children().getElement(1).get_Rect();
            
            A_counter=uint16(1);
            B_counter=uint16(1);
            
            %% pick next
            maxIter= numChildLeft;
            for k=1:maxIter
                
                %% cases where we have to put all remaining nodes in one group
                if (numChildLeft+A_counter) == obj.minNum
                    for j=1:numChildLeft
                        
                        % get Element
                        nextElement = Children_old.getElement(1);
                        nextElement.removeThisElement();
                        Children_A.appendChild(nextElement);
                        A_counter=A_counter+1;
                        new_Rect = GroupA_Rect.enlargeForRectangle(nextElement.get_Rect());
                        %                     GroupA_Rect.delete();
                        GroupA_Rect = new_Rect;
                    end
                    break
                end
                if (numChildLeft+B_counter) == obj.minNum
                    for j=1:numChildLeft
                        
                        % get Element
                        nextElement = Children_old.getElement(1);
                        nextElement.removeThisElement();
                        Children_B.appendChild(nextElement);
                        B_counter=B_counter+1;
                        new_Rect = GroupB_Rect.enlargeForRectangle(nextElement.get_Rect());
                        %                     GroupB_Rect.delete();
                        GroupB_Rect = new_Rect;
                    end
                    break
                end
                
                
                %  PN1
                Delta_d = zeros(3,numChildLeft); % first row is max Delta, 2nd and 3rd row are respective values
                
                candidateElement = Children_old;
                for j=1:numChildLeft
                    candidateElement = candidateElement.get_Succ();
                    candidateRect  = candidateElement.get_Rect();
                    
                    newGroupA_Rect = GroupA_Rect.enlargeForRectangle(candidateRect);
                    newGroupB_Rect = GroupB_Rect.enlargeForRectangle(candidateRect);
                    dA = newGroupA_Rect.get_area() - GroupA_Rect.get_area();
                    dB = newGroupB_Rect.get_area() - GroupB_Rect.get_area();
                    Delta_d(1,j) = abs(dA-dB);
                    Delta_d(2,j) = dA;
                    Delta_d(3,j) = dB;
                    
                end
                % PN2
                [~, pickIndex] = max(Delta_d(1,:));
                
                
                if Delta_d(2,pickIndex) == Delta_d(3,pickIndex)
                    if GroupA_Rect.get_area() == GroupB_Rect.get_area()
                        if A_counter < B_counter
                            choose_A = 1;
                        else
                            choose_A = 0;
                        end
                    else
                        if GroupA_Rect.get_area() < GroupB_Rect.get_area()
                            choose_A = 1;
                        else
                            choose_A = 0;
                        end
                    end
                else
                    if Delta_d(2,pickIndex) < Delta_d(3,pickIndex)
                        choose_A = 1;
                    else
                        choose_A = 0;
                    end
                end
                
                if choose_A
                    
                    % append to A List
                    A_counter=A_counter+1;
                    newPick = Children_old.getElement(pickIndex);
                    newPick.removeThisElement();
                    Children_A.appendChild(newPick);
                    numChildLeft=numChildLeft-1;
                    
                    % enlarge Rectangle
                    new_Rect = GroupA_Rect.enlargeForRectangle(newPick.get_Rect());
                    %                     GroupA_Rect.delete();
                    GroupA_Rect = new_Rect;
                    
                    
                else
                    % append to B List
                    B_counter=B_counter+1;
                    newPick = Children_old.getElement(pickIndex);
                    newPick.removeThisElement();
                    Children_B.appendChild(newPick);
                    numChildLeft=numChildLeft-1;
                    
                    % enlarge Rectangle
                    new_Rect = GroupB_Rect.enlargeForRectangle(newPick.get_Rect());
                    %                     GroupB_Rect.delete();
                    GroupB_Rect = new_Rect;
                    
                    
                end
            end
            
            
            % now we have split into nodes Children_A and Children_B
            

            Children_A.set_Rect(GroupA_Rect);
            Children_B.set_Rect(GroupB_Rect);
            Children_A.set_numChild(A_counter);
            Children_B.set_numChild(B_counter);
                       
            
            if obj.isLeaf == 1
                Children_A.set_isLeaf(1);
                Children_B.set_isLeaf(1);
                
            end
            
            if obj.isRoot == 0
                ParentNode =  obj.Parent;
                obj.removeThisElement();
                obj.toDelete= 1;
                obj = Children_A;
                ParentNode.appendChild(Children_A);
                ParentNode.appendChild(Children_B);
                %                 obj.delete();
            else
                obj.Children = RNode();
                obj.numChild = 0; % reset, each appendChild adds one
                obj.appendChild(Children_A);
                obj.appendChild(Children_B);
                obj.adjustNode();
                obj.isLeaf = 0;
            end
        end
        
        % NodeList of type MyList with Content as pointer to RNode
        function NodeArray = depthSearchLanelets(obj,thePoint,NodeArray)
            
            if obj.get_Rect().containsPoint(thePoint)
                
                if obj.numChild >0
                    Iterator = obj.Children;
                    while Iterator.hasSucc()
                        Iterator = Iterator.get_Succ();
                        NodeArray = Iterator.depthSearchLanelets(thePoint,NodeArray);
                        
                    end
                else % real Leaf
                    lanelet_polygon = obj.Content.get_polygon();
                    if inpolygon(thePoint.get_x,thePoint.get_y,lanelet_polygon(1,:),lanelet_polygon(2,:))                
%                     newNode = MyList(obj);
%                     NodeList = NodeList.appendElement(newNode);
                      NodeArray = [NodeArray, obj.Content.get_id()];
                    end
                end
            else
            end
        end
        
        function NodeList = depthSearchRectangles(obj,thePoint,NodeList)
            
            if obj.get_Rect().containsPoint(thePoint)
                
                if obj.numChild >0
                    Iterator = obj.Children;
                    while Iterator.hasSucc()
                        Iterator = Iterator.get_Succ();
                        NodeList = Iterator.depthSearchRectangles(thePoint,NodeList);
                        
                    end
                else % real Leaf
                    newNode = MyList(obj);
                    NodeList = NodeList.appendElement(newNode);  
                end
            else
                1;
                % do nothing, not a candidate
            end
        end
       
        function LeafNode = chooseLeaf(obj,newNode)
            newRectangle = newNode.get_Rect();
            eps = 1e-2; % threshold from which rectangle areas are not considered equal
            % TreeCells ist Cell Array with all Nodes
            if obj.isLeaf
                LeafNode= obj;
            else
                
                % calculate area of for enlargement
                areaNeeded = zeros(1,obj.numChild);
                ChildList = obj.Children; % go to root of list.
                for k=1:obj.numChild
                    if ChildList.hasSucc() ==1
                        ChildList= ChildList.get_Succ();   
                        
                        areaNeeded(k) = ChildList.get_Rect().enlargeForRectangle(newRectangle).get_area();
                    else
                        break;
                    end
                end
                min_area = min(areaNeeded);
                equalAreaLogical = (areaNeeded< (min_area + eps)); % logical array with entries = 1 if rectangle area are minimum
                numCandidates = sum(equalAreaLogical);
                
                % choose the one with the least current Area
                currArea = zeros(2,numCandidates); % store area (1) and index (2) of current rectangle
                ChildList = obj.Children; % initialize with head
                ChildList = ChildList.get_Succ(); % go to first ListElement after root
                counter = 1;
                
                while numCandidates > 0
                    if equalAreaLogical(counter)==1
                        
                        currArea(1,numCandidates)=(ChildList.get_Rect().get_area());
                        currArea(2,numCandidates) = counter;
                        numCandidates= numCandidates-1;
                        counter=counter+1;
                        if ChildList.hasSucc() == 1
                            ChildList = ChildList.get_Succ();
                        end
                        continue;
                    else
                        
                        if ChildList.hasSucc() == 1
                            ChildList = ChildList.get_Succ();
                        else
                            error('implementation error: numCandidates inconsistent');
                        end
                        counter=counter+1;
                    end
                end
                
                [~,min_Index] = min(currArea(1,:));
                
                
                
                LeafIndex=currArea(2,min_Index(1)); % take first if many different exist
                
                
                searchNode = obj.Children.getElement(LeafIndex);
                LeafNode = searchNode.chooseLeaf(newNode);
            end
            
            
        end
        
        %% drawing
        function drawNode(obj)
            newRect = obj.get_Rect();
            lle = newRect.get_lle();
            rue = newRect.get_rue();
            x = lle.get_x();
            y = lle.get_y();
            h = rue.get_y() - lle.get_y();
            w =  rue.get_x() - lle.get_x();
            
            figure
            rectangle('Position',[x,y,w,h],'EdgeColor','red','LineStyle','-'); % not a leaf
            
        end
        
        
        
        function drawTree(obj)
            
            if obj.numChild > 0
                Iterator = obj.Children;
                while Iterator.hasSucc()
                    Iterator = Iterator.get_Succ();
                    Iterator.drawTree();
                end
            end
            newRect = obj.get_Rect();
            lle = newRect.get_lle();
            rue = newRect.get_rue();
            x = lle.get_x();
            y = lle.get_y();
            h = rue.get_y() - lle.get_y();
            w =  rue.get_x() - lle.get_x();
            
            fontsize = 10;
            if obj.isRoot
                
                rectangle('Position',[x,y,w,h],'EdgeColor','black','LineStyle','--'); % not a leaf
                text(x+0.05,y+0.05,num2str(obj.id),'Color','black','FontSize',fontsize);
            else
                if obj.isLeaf == 0
                    
                    if obj.hasContent == 1
                        rectangle('Position',[x,y,w,h],'EdgeColor','red','LineStyle','--');
                        text(x,y+0.05,num2str(obj.id),'Color','red','FontSize',fontsize);
                    else
                        
                        rectangle('Position',[x,y,w,h],'EdgeColor','blue','LineStyle','--'); % not a leaf
                         text(x+0.05,y+0.05,num2str(obj.id),'Color','blue','FontSize',fontsize);
                    end
                    
                else
                    rectangle('Position',[x,y,w,h],'EdgeColor','green','LineStyle','--'); % a leaf
                    text(x,y+0.05,num2str(obj.id),'Color','green','FontSize',fontsize);
                end
            end
            
        end
        
        
        
        %% get and set function
        
        function dl = get_toDelete(obj)
            dl = obj.toDelete;
        end
        
        function out = get_Rect(obj)
            out=obj.Rect;
        end
        function pg = get_Polygon(obj)
            pg=obj.Polygon;
        end
        function out = get_isLeaf(obj)
            out=obj.isLeaf;
        end
        function out = get_Content(obj)
            out=obj.Content;
        end
        function out = get_id(obj)
            out=obj.id;
        end
        function out = get_isRoot(obj)
            out=obj.isRoot;
        end
        function out = get_Parent(obj)
            out=obj.Parent;
        end
        function Children = get_Children(obj)
            Children = obj.Children();
        end
%         function out = get_maxNum(obj)
%             out=obj.maxNum;
%         end
%         function out = get_minNum(obj)
%             out=obj.minNum;
%         end
%         function out = get_numChild(obj)
%             out=obj.numChild;
%         end
        
        
        function obj = set_numChild(obj,value)
            obj.numChild=value;
        end  
        function obj = set_Rect(obj,newRectangle)
            obj.Rect = newRectangle;
        end
%         function obj = set_Polygon(obj,newPolygon)
%             obj.Polygon = newPolygon;
%         end
%         function obj =  set_id(obj,value)
%             obj.id=value;
%         end
        function obj = set_isLeaf(obj,value)
            if value > 0
                obj.isLeaf = 1;
            else
                obj.isLeaf = 0;
            end
        end
%         function obj = set_isRoot(obj,value)
%             if value > 0
%                 obj.isRoot = 1;
%             else
%                 obj.isRoot = 0;
%             end
%         end
        function obj = set_Parent(obj,new_Parent)
            obj.Parent=new_Parent;
        end
%         function  obj = set_Children(obj,newChildren)
%             obj.Children=newChildren;
%         end 
%         function obj =  set_Container(obj,newContainer)
%             obj.Container = newContainer;
%         end
        
        
        
        function Pred = get_Pred(obj)
            Pred = obj.Pred;
        end
        
        % sets a new predecessor. set_Pred(obj) to delete predecessor
        function obj = set_Pred(obj,newPred)
            if nargin ==2
                obj.Pred = newPred;
                obj.hasPredElement = 1;
            else
                obj.Pred = [];
                obj.hasPredElement = 0;
            end
            
        end
        function Succ = get_Succ(obj)
            Succ = obj.Succ;
        end
        % sets a new successor. set_Pred(obj) to delete successor
        function obj = set_Succ(obj,newSucc)
            if nargin ==2
                obj.Succ= newSucc;
                obj.hasSuccElement = 1;
            else
                obj.Succ = [];
                obj.hasSuccElement = 0;
            end
        end
        
        function out = hasSucc(obj)
            out = obj.hasSuccElement;
        end
        function out = hasPred(obj)
            out = obj.hasPredElement;
        end
        
    end
    methods (Access = private)
    %% function for list behaviour, used to manage child list
        function obj = appendElement(obj,newRNode)
            if obj.hasSuccElement  == 0
                obj.Succ = newRNode;
                obj.hasSuccElement = 1;
                newRNode.set_Pred(obj);
                if newRNode.hasSucc()
                    newRNode.set_Succ();
                end
            else
                thisSucc=obj.Succ;
                appendElement(thisSucc,newRNode);
            end
            
        end
        
        function removeThisElement(obj)
            if  obj.hasPred == 0
                error('cannot be removed because this node has no predecessor');
            else
                
                if obj.hasSucc
                    thisSucc = obj.Succ;
                    thisPred = obj.Pred;
                    thisSucc.set_Pred(thisPred);
                    thisPred.set_Succ(thisSucc);
                else
                    % tail
                    thisPred = obj.Pred;
                    thisPred.set_Succ();
                end
                obj.Parent.decrement_numChild();
            end
        end
        function deleteThisElement(obj)
            removeThisElement(obj);
            obj.delete(); % destructor
        end
        % removes Node with distance dist from current Node
        function removeByIndex(obj,dist)
            removalObj = obj.getElement(dist);
            removeThisElement(removalObj);
        end
        function deleteByIndex(obj,dist)
            deletionObj = obj.getElement(dist);
            deleteThisElement(deletionObj);
        end
        %% returns Element #index starting from obj. 0 is obj
        function result=getElement(obj,index)
            
            if index < 0
                error('invalid index in MyList.getElement()');
            end
            if index==0
                result=obj;
            else
                if obj.hasSucc
                    thisSucc = obj.Succ;
                    result = thisSucc.getElement(index-1);
                else
                    % reached end of list
                    error('indexOutOfBound in RNode.getElement');
                end
                
            end
        end
        
        
        
        function obj=increment_numChild(obj)
            obj.numChild = obj.numChild + 1;
        end
        function obj=decrement_numChild(obj)
            obj.numChild = obj.numChild - 1;
        end
end
end

