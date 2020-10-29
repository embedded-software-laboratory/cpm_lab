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

function main
    %% Create lane_graph.mat
    close all
    addpath('../map_print2');
    assert(isfile('../map_print2/cache.mat'));
    if (~isfile('../map_print2/lane_graph.mat'))
        cur_dir = pwd;
        cd('../map_print2');
        lane_graph();
        cd(cur_dir);
    end
    org = load('lane_graph.mat');
    original_lane_graph = org.lane_graph;
    visualize_lane_graph(original_lane_graph);
    
    %% delete nodes
%     outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
%     inner_cross_nodes = [37,  40, 28, 14, 11,57,  24, 68, 46, 75, 78,  66, 48, 23, 42,  60,  61, 27, 16, 17];
%     comb = union(outer_circle_nodes, inner_cross_nodes);
%     steep_curve_edges = [7 13; 58 55; 49 62; 79 69; 73 77; 38 33; 29 43; 19 2];

comb = [47, 65];
%     inner_cross_nodes = [37,  40, 28, 14, 11,57,  24, 68, 46, 75, 78,  66, 48, 23, 42,  60,  61, 27, 16, 17];
%     comb = union(outer_circle_nodes, inner_cross_nodes);
     steep_curve_edges = [47 67];
        
    laneGraph = sub_graph_deletion(original_lane_graph, comb, steep_curve_edges);
    visualize_lane_graph(laneGraph);
    lane_graph_to_cpp(laneGraph)
       
    %% select nodes
%     outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
%     
%     laneGraph = sub_graph_selection(original_lane_graph, outer_circle_nodes);
%     visualize_lane_graph(laneGraph);
%     lane_graph_to_cpp(laneGraph);
    
    %% invert graph
%     inverted_laneGraph = graph_inversion(laneGraph);
%     lane_graph_to_cpp(inverted_laneGraph);
end