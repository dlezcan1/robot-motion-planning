%% graphsearch_dijkstra.m
% 
% this is a function to perform Dijkstra's algorithmic graph search
% on an undirected graph
% 
% This algorithm follows pseudocode provided in the class.
%
% Args:
%   - n x n weighted adjacency matrix
%
% Returns:
%   - shortest path

%% Main Funciton
function path = graphsearch_dijkstra(adj_mat)
    %% Arguments Block
    arguments
        adj_mat (:,:) {mustBeSquare(adj_mat)};
    end
    
    %% Algorithm 
    
end



%% Helper functions
function mustBeSquare(x)
    if ~isequal(size(x,1),size(x,2))
        eid = 'Size:notSquare';
        msg = 'Size of matrix is not square.';
        throwAsCaller(MException(eid,msg))
    end
end