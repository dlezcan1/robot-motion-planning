%% Dijkstra.m
% 
% this is a function to perform Dijkstra's algorithmic graph search
% on an undirected graph
% 
% This algorithm follows pseudocode provided in the class.
%
% Args:
%   - n x n weighted adjacency matrix
%   - initial position index in the adjacency matrix
%   - goal position index in the adjacency matrix
%
% Returns:
%   - shortest path

%% Main Funciton
function [dist_goal, prev_goal, path] = Dijkstra(adj_mat, x_init, x_goal)
    %% Arguments Block
    arguments
        adj_mat (:,:) {mustBeSquare(adj_mat)};
        x_init {mustBeInteger, mustBeInRange(x_init, adj_mat)};
        x_goal {mustBeInteger, mustBeInRange(x_goal, adj_mat)};
    end
    
    %% Set-Up
    % initialize the distances and previous values
    dist = inf * ones(1, size(adj_mat, 2));
    prev = -1 * ones(size(dist));
    
    % initialize initial distance
    dist(x_init) = 0;
    
    % initialize unvisited nodes
    U = 1:length(adj_mat);
    
    %% Algorithm
    while ismember(x_goal, U)
        % find value with smallest distance
        [~, idx_min] = min(dist(U));
        C = U(idx_min);
        U(idx_min) = []; % remove the closest value from unvisited nodes
        
        % find the neigbors of C
        neighbors = find(adj_mat(C, [1:C-1, C+1:end]) < inf);
        
        % iterate through the neighbors
        for v = neighbors
            alt = dist(C) + adj_mat(C, v); % total distance
            if alt < dist(v) % set new closest
                dist(v) = alt;
                prev(v) = C;
            end
        end     
    end
    
    %% Return arguments
    % in the pseudocode
    dist_goal = dist(x_goal);
    prev_goal = prev(x_goal);
    
    % the path to send
    path = [x_goal];
    while path(1) ~= x_init 
        if prev(path(1)) <= 0
            path = -1;
            break
        else
            path = [ prev(path(1)), path ];
        end
    end
    
end



%% Helper functions
function mustBeSquare(x)
    if ~isequal(size(x,1),size(x,2))
        eid = 'Size:notSquare';
        msg = 'Size of matrix is not square.';
        throwAsCaller(MException(eid,msg))
    end
end

function mustBeInRange(x, adj_mat)
    if x > size(adj_mat, 1)
        eid = 'Size:indexOutOfRange';
        msg = 'Position is out of range.';
        throwAsCaller(MException(eid,msg))
    end
end