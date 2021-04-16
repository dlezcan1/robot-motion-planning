%% vertical_cell_decomposition.m
%
% this is a function file to create the vertical cell decomposition
% of a 2D rigid-body robot moving in the plane
%
% - written by: Dimitri Lezcano

function [V, E, n_init, n_goal, G] = vertical_cell_decomposition(qI, qG, CB, bounds)
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
           
    
    
    %% Return values
    n_init = find(all(G.Nodes.q == qI',2), 1);
    n_goal = find(all(G.Nodes.q == qG',2), 1);
    V = G.Nodes.q';
    E = full(G.adjacency);

end

%% Helper function
function S_int = find_intersections(S)
    arguments
        S cell;
    end
    
    % initialization
    Q = {};
    T = struct();
    
    % sort the line segments by their left-most point (left endpoints)
    min_x_S = inf * ones(1,numel(S));
    for i = 1:numel(S)
        S_i = S{i};
        [~, sidx] = sort(S_i(1,:), 'ascend');
        S{i} = S_i(:,sidx);
        min_x_S(i) = S_i(1,sidx(1));
    end
    
    [~, sidx] = sort(min_x_S, 'ascend');
    S = S{sidx}; % sort the obstacles
    
    % insert into Q
    Q = {};
    for i = 1:numel(S)
        S_i = S{i};
        
        % insert left endpoint as whole segment
        Q{end+1} = S_i;
        
        % if different right x-coord, insert right endpoint individually
        
        
    
    
    
        
    
    
   
end
