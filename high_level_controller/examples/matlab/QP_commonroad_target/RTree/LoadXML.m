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

function commonroad_data = LoadXML(filepath)
%% output structs
%     commonroad_data.Obstacle_Data = Obstacle_Data;
%     commonroad_data.Planning_Data = Planning_Data;
%     commonroad_data.r_tree        = r_tree;
%     commonroad_data.map           = rb_tree;
%     commonroad_data. dt           = commonroad_dt;
%     commonroad_data.succGraph     = succGraph;

%% 
Planning_Data = struct;
Obstacle_Data = struct;
numObst = 0;
numPlanning =0;
Obstacle_Data.Obstacle = {};
    path = './Maps/';
    if nargin < 1
        filename = 'LabMapCommonRoad';
%              filename = 'scenarios_cooperative_C-DEU_B471-1_1_T-1';
%     filename = 'scenarios_cooperative_C-USA_Lanker-2_2_T-1';
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

%         % read input for debugging
%         save([filepathNoType,'Raw','.mat'],'CommonRoadScenario');

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
    succGraph=digraph;
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
            
            % build nodes (edges later as we need all nodes)
            succGraph = addnode(succGraph,num2str(newId));
        end
        if  strcmp(CommonRoadScenario.Children(k).Name,'obstacle')
            newObstacleNode = CommonRoadScenario.Children(k);
            numObst = numObst +1;
            Obstacle_Data.Obstacle{1,numObst} = Obstacle(newObstacleNode);
            
          
            
        end
        if  strcmp(CommonRoadScenario.Children(k).Name,'planningProblem')
            newPlanningNode = CommonRoadScenario.Children(k);
            numPlanning = numPlanning +1;
            Planning_Data.Plans{1,numPlanning} = PlanningProblem(newPlanningNode);
            
            
        end
    end
    

    % insert edges and nodes in successor graph
    % iterate through tree
    Iterator = rb_tree.Minimum();
    while ~isnan(Iterator)

            SuccList = Iterator.value.get_SuccList();
            numSucc = Iterator.value.get_numSucc();
            
            weight = sum(Iterator.value.get_centerlineDist()); % unweighted graph now
            for k=1:numSucc
                succGraph = addedge(succGraph,num2str(get_id(Iterator.value)),num2str(SuccList(k)),weight);
            end
            Iterator = rb_tree.NextLargest(Iterator);
    end
    
    
    Obstacle_Data.numObst = numObst;
    Planning_Data.numPlanning = numPlanning;
    toc

    commonroad_data.Obstacle_Data = Obstacle_Data;
    commonroad_data.Planning_Data = Planning_Data;
    commonroad_data.r_tree        = r_tree;
    commonroad_data.map           =rb_tree;
    commonroad_data. dt           = commonroad_dt;
    commonroad_data.succGraph     = succGraph;


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