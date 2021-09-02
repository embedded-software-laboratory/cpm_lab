function lane_graph

    cache = load('cache');
    
    % convert to cartesian form: [x, y, cos(yaw), sin(yaw)]
    f_names = fieldnames(cache);
    for i = 1:length(f_names)
        f_name = f_names{i};
        value = cache.(f_name);
        if startsWith(f_name,'segment')
            cache.(f_name) = cartesian_nodes(value.nodes);
        elseif startsWith(f_name,'node')
            cache.(f_name) = cartesian_nodes(value);
        end
    end
        
    segment_A = cache.segment_A;
    segment_B = cache.segment_B;
    segment_C = cache.segment_C;
    segment_D = cache.segment_D;
    segment_E = cache.segment_E;
    segment_F = cache.segment_F;
    segment_G = cache.segment_G;
    segment_H = cache.segment_H;
    segment_J = cache.segment_J;
    
    clf
    hold on
    axis equal
    grid on
    
    % Transfer a bit of segment A to B, this simplifies the routing graph
    [M,I] = min(vecnorm((segment_A - cache.node_transition_ABEF)'));
    assert(M < 1e-8);
    segment_B = [segment_A(I:end-1,:); segment_B];
    segment_A = segment_A(1:I,:);
    
    
    % Split A at D merge
    parts = split_segment(segment_A, cache.node_transition_AD);
    segment_A1 = parts{1};
    segment_A2 = parts{2};
    
    % Split B at C merge
    parts = split_segment(segment_B, cache.node_transition_BC);
    segment_B1 = parts{1};
    segment_B2 = parts{2};
    
    % Split C
    parts = split_segment(segment_C, segment_C(end/2,:));
    segment_C1 = parts{1};
    segment_C2 = parts{2};
    
    % Split D
    parts = split_segment(segment_D, segment_D(end/2,:));
    segment_D1 = parts{1};
    segment_D2 = parts{2};
    
    segments = {};
    
    % handle double lanes with switch
    segments_switch = {segment_A1, segment_A2, segment_B1, segment_B2, segment_C1, segment_C2, segment_D1, segment_D2};
    segments = [segments double_segment_with_switch(segment_A1,  1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_A2,  1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_B1,  1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_B2,  1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_C1, -1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_C2, -1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_D1, -1, cache.lane_width)];
    segments = [segments double_segment_with_switch(segment_D2, -1, cache.lane_width)];
    
    % intersection
    segments = [segments {expand_intersection_segment(segment_H, cache.lane_width, -1, false)}];
    segments = [segments {expand_intersection_segment(segment_G, cache.lane_width, -1, false)}];
    segments = [segments {expand_intersection_segment(segment_J, cache.lane_width, -1, false)}];
    segments = [segments {expand_intersection_segment(segment_G, cache.lane_width, 1, true)}];
    segments = [segments {expand_intersection_segment(segment_G, cache.lane_width, 2, true)}];
    
    
    % add other lanes
    segments = [segments, {segment_E, segment_F, segment_J, segment_G, segment_H}];
    
    
    % mirroring right to left
    n_base_segments = length(segments);
    for i = 1:n_base_segments
        s = segments{i};
        s = s .* [-1 1 -1 1];
        s = reverse_segment(s);
        segments = [segments, {s}];
    end
    
    % mirroring top to bottom
    n_base_segments = length(segments);
    for i = 1:n_base_segments
        s = segments{i};
        s = s .* [1 -1 1 -1];
        s = reverse_segment(s);
        segments = [segments, {s}];
    end
    
    % fix coordinate system, offset everything
    for i = 1:length(segments)
        s = segments{i};
        segments{i} = s + [4.5/2 2 0 0];
    end
    
    lane_graph = build_routing_graph(segments);
    lane_graph_to_cpp(lane_graph);
    save lane_graph lane_graph
    
    for i = 1:length(segments)
        plot(segments{i}(:,1), segments{i}(:,2),'-', 'LineWidth',2)
        k = floor(size(segments{i}, 1)/3);
        R = [segments{i}(k,3) -segments{i}(k,4);segments{i}(k,4) segments{i}(k,3)];
        p = 0.01 * R * [0 1; 3 0; 0 -1]';
        plot(p(1,:)+segments{i}(k,1), p(2,:)+segments{i}(k,2),'r-', 'LineWidth',2)
    end
    
    
    axis equal
    grid on
    xlim([0 4.5])
    ylim([0 4])
end

function routing_graph = build_routing_graph(segments)

    nodes = nan(0,4);

    for i = 1:length(segments)
        nodes(end+1, :) = segments{i}(1,:);
        nodes(end+1, :) = segments{i}(end,:);
    end
    
    node_diff = permute(nodes, [1 3 2]) - permute(nodes, [3 1 2]);
    node_dist = vecnorm(node_diff, 2, 3) + 10 * eye(size(nodes,1));
    
    threshold = 0.05;
    [I,J] = find(node_dist < threshold);
    assert(~any(I==J));
    
    node_clusters = {};
    for k = 1:length(I)
        joined = false;
        for p = 1:length(node_clusters)
            if any(any(node_clusters{p} == [I(k); J(k)]))
                node_clusters{p} = [node_clusters{p} I(k) J(k)];
                joined = true;
            end
        end
        
        if ~joined
            node_clusters{end+1} = [I(k) J(k)];
        end
    end

    true_nodes = nan(0,4);
    for p = 1:length(node_clusters)
        true_nodes(end+1,:) = mean(nodes(unique(sort(node_clusters{p})), :));
    end
    
    routing_graph = struct;
    routing_graph.nodes = true_nodes;
    routing_graph.edges = struct('start_node_index', {}, 'end_node_index', {}, 'path', {});
    
    
    for i = 1:length(segments)
        
        [M,I] = min(vecnorm(routing_graph.nodes - segments{i}(1,:),2,2));
        assert(M < threshold);
        routing_graph.edges(i).start_node_index = I;
        
        
        [M,I] = min(vecnorm(routing_graph.nodes - segments{i}(end,:),2,2));
        assert(M < threshold);
        routing_graph.edges(i).end_node_index = I;
        
        
        routing_graph.edges(i).path = segments{i};
    end
    
end

function segment = expand_intersection_segment(segment, lane_width, steps, reverse)

    displacement = lane_width * [-segment(:,4) segment(:,3)];
    displacement(:,3:4) = 0;
    
    segment = segment + steps * displacement;
    
    if reverse
        segment = reverse_segment(segment);
    end
    
    segments = fix_direction_vectors({segment});
    segment = segments{1};

end

function segments = double_segment_with_switch(segment, direction, lane_width)

    displacement = direction * lane_width * [-segment(:,4) segment(:,3)];
    displacement(:,3:4) = 0;
    
    f = ((cos(linspace(0,pi, size(segment, 1)))+1)/2)';

    segments{1} = segment;
    segments{2} = segment + displacement;
    segments{3} = segment + f .* displacement;
    segments{4} = segment + (1-f) .* displacement;
    
    segments = fix_direction_vectors(segments);

end

function segment = reverse_segment(segment)
    segment = segment .* [1 1 -1 -1];
    segment = flipud(segment);
end

function segments = fix_direction_vectors(segments)
    for i = 1:length(segments)
        dx = diff(segments{i}(:,1));
        dy = diff(segments{i}(:,2));
        len = sqrt(dx.^2 + dy.^2);
        segments{i}(2:end,3) = dx ./ len;
        segments{i}(2:end,4) = dy ./ len;
        
        segments{i}(1,3) = segments{i}(2,3);
        segments{i}(1,4) = segments{i}(2,4);
    end
end


function L = segment_length(segment)
    L = sum(sqrt(diff(segment(:,1)).^2+diff(segment(:,2)).^2));
end

function parts = split_segment(segment, splitting_nodes)

    idx = [1 size(segment, 1)];

    for i = 1:size(splitting_nodes, 1)
        [M,I] = min(vecnorm((segment - splitting_nodes(i,:))'));
        assert(M < 1e-4);
        idx(end+1) = I;
    end
    
    idx = unique(sort(idx));
    assert(numel(idx) >= 3)
    parts = {};
    
    for i = 2:numel(idx)
        slice = idx(i-1):idx(i);
        assert(numel(slice) > 1);
        parts{end+1} = segment(slice, :);
    end
 
    
end

function S2 = cartesian_nodes(S)
    S2 = [S(:,1:2) cos(S(:,3)) sin(S(:,3))];
end