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

% function to load the commonroad lanelet data into the a struct.
function commonroad_data = LoadXML(filepath)
%% output structs
%     commonroad_data.r_tree        = r_tree;
%     commonroad_data.map           = rb_tree;
%     commonroad_data. dt           = commonroad_dt;

%% 

    path = './Maps/';
    if nargin < 1
        filename = 'LabMapCommonRoad';
        filepath = [path,filename,'.xml'];
    end
    if ~strcmp(filepath(end-3:end),'.xml')
       error('CommonRoad File does not end with .xml'); 
    end
    filepathNoType = filepath(1:end-4);
    if ~isfile([filepathNoType,'.mat'])
        disp('start reading XML');
        tic
        
        CommonRoadScenario=parseXML(filepath); % matlab function whichs reads everything

        CommonRoadScenario.Children=removeEmptyNodes(CommonRoadScenario.Children);
        toc





        save([filepathNoType,'.mat'],'CommonRoadScenario');

    else
        disp('load XML from Cache');
        tic
        load([filepathNoType,'.mat']);
        toc

    end

    numAttributes = size(CommonRoadScenario.Attributes,2);
    for att = 1:numAttributes
       switch CommonRoadScenario.Attributes(att).Name
           case 'timeStepSize'
               commonroad_dt = str2double(CommonRoadScenario.Attributes(att).Value);
           
       end
    end
    

    %% build the tree
    disp ('building trees');
    tic
    r_tree=RTree();
    rb_tree = RedBlackTree();
    numEntries = size(CommonRoadScenario.Children,2);
    for k = 1:numEntries
        if  strcmp(CommonRoadScenario.Children(k).Name,'lanelet')
            newLaneletNode = CommonRoadScenario.Children(k);

            %% create lanelet object
            newLanelet = Lanelet(newLaneletNode);
            lanelet_id = newLanelet.get_id();
            newRect = newLanelet.get_MBR();
            newId = r_tree.numNodes +1;
            newContent = newLanelet;
            rootNode =[]; % set in r_tree.insert()
            % insert in RTree
            r_tree.insert(RNode(newRect,newId,rootNode,newContent,0));

            % insert in RedBlackTree as key-value-map
            rb_tree.Insert(lanelet_id,newLanelet);
            
        end
      
    end
    

   
    
    

    commonroad_data.r_tree        = r_tree;
    commonroad_data.map           =rb_tree;
    commonroad_data. dt           = commonroad_dt;



end


% remove those empty 'text' line nodes from parseXML
function condensedNode=removeEmptyNodes(Node)
counter=1;
condensedNode=struct;
for k=1:size(Node,2)
    if (~strcmp(Node(k).Name, '#text'))
        condensedNode(counter).Name = Node(k).Name;
        if ~isempty(Node(k).Attributes)
            condensedNode(counter).Attributes = Node(k).Attributes;
        end
        if ~isempty(Node(k).Data)
            condensedNode(counter).Data = Node(k).Data;
        end
        if ~isempty(Node(k).Children)
            condensedNode(counter).Children = (removeEmptyNodes(Node(k).Children));
        end
        counter=counter+1;
    else
        if ~isempty(strtrim(Node(k).Data))
            condensedNode(counter).Data=Node(k).Data;
            counter=counter+1;
        end
        
    end
    
end
end