%% Astar.m
%
% this ia function to perform A* graph search algorithm on an adjacency matrix
%
% - written by: Dimitri Lezcano

%% Main Function
function path = Astar(adj_mat, x_init, x_goal, kwargs)
    %% Arguments Block
    arguments
        adj_mat (:,:) {mustBeSquare(adj_mat)};
        x_init {mustBeInteger, mustBeInRange(x_init, adj_mat)};
        x_goal {mustBeInteger, mustBeInRange(x_goal, adj_mat)};
        kwargs.heuristic {mustBeMember(kwargs.heuristic, {'num_nodes', 'row_sum'})} = 'num_nodes';
    end

    %% Set-Up
    % heuristic function
    if strcmp( kwargs.heuristic, 'num_nodes')
        heur = @ num_nodes;
    elseif strcmp(kwargs.heuristic, 'row_sum')
        heur = @ row_sum;
    end
    
    % the open and closed ques
    O = [x_init]; % priority queue (open)
    C = [];       % closed nodes
    
    % the previous values and costs
    prev = -1 * ones(1, size(adj_mat, 2));
    f = inf * ones(size(prev));
    g = inf * ones(size(prev));
    
    % initilize initial state
    g(x_init) = 0;
    f(x_init) = heur(adj_mat, x_init, x_goal);
    
    %% Algorithm
    while ~isempty(O)
        % pick the best from O
        [~, idx_best] = min(f(O));
        x_best = O(idx_best);
        O(idx_best) = []; % remove from open
        C = [C, x_best]; % add to closed
        
        % check if at the goal state
        if x_best == x_goal
            break
        end
        
        % find the neighbors
        neighbors = find(adj_mat(x_best, :) < inf);
        
        % iterate through the neighbors
        for x = neighbors
            % check if already closed
            if ismember(x, C)
                continue
            end
            
            % find best next step
            g_temp = g(x_best) + adj_mat(x_best, x);
            if ~ismember(x, O) % add to open set
                O = [O, x];
            elseif g_temp >= g(x)
                continue
            end
            
            % set the previous values
            prev(x) = x_best;
            g(x) = g_temp;
            f(x) = g(x) + heur(adj_mat, x, x_goal);
        end
        
    end
    
    %% Return path
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


%% Helper Functions
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

% number of nodes heuristic
function dist = num_nodes(adj_mat, x_start, x_end)
    dist = Dijkstra(adj_mat > 0 & adj_mat < inf, x_start, x_end);


end

function dist = row_sum(adj_mat, x_start, x_end)
    dist = sum(adj_mat(x_start, :) * (adj_mat < inf));

end
