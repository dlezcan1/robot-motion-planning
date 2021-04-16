%% vertical_cell_decomposition_brute.m
%
% this is a function file to create the vertical cell decomposition
% of a 2D rigid-body robot moving in the plane
%
% - written by: Dimitri Lezcano

function [V, E, n_init, n_goal, G] = vertical_cell_decomposition_brute(qI, qG, CB, bounds)
	%% Arguments Block
	arguments
		qI (2,1);
		qG (2,1);
		CB cell;
		bounds (2,4);
	end
	
	%% Initialization
    % graph
    G = addnode(graph, 2);
    G.Nodes.q = [qI, qG]';
           
    % add boundary mid points to the graph
    G = addnode(G, 2);
    G.Nodes.q(3:4,:) = [mean(bounds(:,[1, 4]), 2), mean(bounds(:,2:3),2)]';
    
    % Grab all of the vertex points
    points = cat(2, CB{:}); % [];
%     for i = 1:length(CB)
%         points = [points, CB{i}];
%     end
    
    % sort the points
    [~, idx] = sort(points(1,:), 'ascend');
    points = points(:, idx);
    
    %% Iterate through the points and find the vertices
    for i = 1:size(points,2)
        % grab the POI
        pt_i = points(:,i);
        x_i = pt_i(1);
        
        % Grab the line segments
        poi_top = vertical_intersection(x_i, bounds(:,3:4));
        poi_bttm = vertical_intersection(x_i, bounds(:,1:2));
        
        % points of intersection with obstacles
        poi_obs = check_obs_intersections(x_i, CB);
        
        % concatenate line segments
        poi = [poi_bttm, poi_obs, poi_top];
        N_segs = size(poi,2) - 1;
        
        for k = 1:N_segs
            seg_k = poi(:,k:k+1);
            
            % check if segment is inside obstacles
            if ~vlineseg_intersect(seg_k, CB)
               % insert midpoint into the graph
               mp_k = mean(seg_k, 2);
               G = addnode(G, 1);
               G.Nodes.q(end,:) = mp_k';
               
            end
        end
    end
    
    %% Connect the visibility graph
    % iterate through all possible pairs and see if straight line intersects
    for i = 1:G.numnodes
        for j = [1:i-1, i+1:G.numnodes]
            intersects = false;
            for k = 1:numel(CB)
                % check intersection
                if isintersect_linepolygon(CB{k}, G.Nodes.q([i,j],:)')
                    intersects = true;
                    break;
                end
            end
            
            if ~intersects
                 G = G.addedge(i,j);
            end
        end
    end
    
    %% Return values
    n_init = find(G.Nodes.q == qI', 1);
    n_goal = find(G.Nodes.q == qG', 1);
    V = G.Nodes.q';
    E = full(G.adjacency);

end

%% Helper function
function does_intersect = lineseg_intersect(obs, line)
    l = linspace(0, 1, 200); % discretize line
    pts_l = line(:,1) + l .* line(:,2);
    does_intersect = false;
    for j = 1:size(pts_l,2)
        test_intersect = inpolygon(pts_l(1,j), pts_l(2,j), obs(1,:), obs(2,:));
        if test_intersect
            does_intersect = true;
            return;
        end
    end
end


function does_intersect = vlineseg_intersect(vlineseg, CB)
   
    mp = mean(vlineseg, 2);
    does_intersect = false;
    for i = 1:numel(CB)
        obs_i = CB{i};
        
        [in, on] = inpolygon(mp(1), mp(2), obs_i(1,:), obs_i(2,:));
        
        if in && ~on
            does_intersect = true;
            return;
            
        elseif on 
            does_intersect = true;
            return;
        end
        
    end
    
    
end

function points = check_obs_intersections(x, CB)
    
    % iterate over obstacles
    points = [];
    for i = 1:length(CB)
        obs_i = CB{i};
        obs_i = [obs_i, obs_i(:,1)];
        
        % iterate over edges
        N_edges = size(obs_i,2)-1;
        for j = 1:N_edges
            vert_poi = vertical_intersection(x, obs_i(:,j:j+1));
            
            if ~isnan(vert_poi)
                points = [points, vert_poi];
            end
        end
    end
   
    % sort by y values and remove duplicates
    points = unique(points', 'rows')';
    
end

function vert_poi = vertical_intersection(x, ln_seg)
    arguments
        x double;
        ln_seg (2,2);
    end
    
    [~, idx] = sort(ln_seg(1,:), 'ascend');
    ln_seg = ln_seg(:,idx);
    
    if ln_seg(1,1) == ln_seg(1,2) 
        if ln_seg(1,1) == x
            vert_poi = ln_seg;
        else
            vert_poi = nan;
        end
        
        return;
    end
    
    y_poi = interp1(ln_seg(1,:), ln_seg(2,:), x, 'linear');
    
    if isnan(y_poi)
        vert_poi = nan;
    else
        vert_poi = [x; y_poi];
    end
    
end