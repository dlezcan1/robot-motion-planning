%% build_PRM.m
%
% function to build PRM.
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   - K: the number of nearest neighbors
%   - obstacles: cell array of 2xN matrices (default is {})
%   - xmax, ymax: Positive doubles representing C-space bounds (default = 1,
%                   xmax)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built PRM


function [path, V, E, G] = build_PRM(qI , qG, NumNodes, K, obstacles, xmax, ymax)
    %% Arguments Block
    arguments
        qI (2,1);
        qG (2,1);
        NumNodes double {mustBeInteger};
        K double {mustBeInteger};
        obstacles cell = {};
        xmax double {mustBePositive} = 1;
        ymax double {mustBePositive} = xmax;
    end
    
    %% Initialization
    G = addnode(graph, 2);
    G.Nodes.q = [qI, qG]';
    
    %% Generate number of nodes
    while G.numnodes < NumNodes
       % generate qrand
       qrand = rand_config(xmax, ymax);
       
       % check for collision
       colliding = collision_check_point(qrand, obstacles);
       if ~colliding
           G = G.addnode(1);
           G.Nodes.q(end,:) = qrand';
       end
    end
    
    %% Build Connectivity
    for j = 1:G.numnodes
        % grab q_j
        q_j = G.Nodes.q(j,:)';
        
        % compute k-NN in Nodes to q_j
        knn_idx = knnsearch(G.Nodes.q([1:j-1, j+1:end],:), q_j', 'K', K, 'Distance', 'euclidean');
        
        % add edge if not already there and collision-free
        for idx = knn_idx
           % grab q_idx
           q_idx = G.Nodes.q(idx,:)';
           
           % check if edge exists
           edge_exists = G.findedge(j, idx);
           if edge_exists
               continue;
           end
                      
           % check for collision
           colliding = collision_check_line([q_j, q_idx], obstacles);
                      
           % add edge between q_j and q_idx
           if ~colliding
               G = G.addedge(j, idx);
           end           
        end        
    end
    
    %% Compute Path from qI to qG
    qG_idx = find(all(G.Nodes.q == qG', 2));
    if isempty(qG_idx)
        path = [];
    else
        idx_path = shortestpath(G, 1,qG_idx(1));
        path = G.Nodes.q(idx_path, :)';
    end
    
    %% Return arguments
    V = G.Nodes.q';
    E = full(G.adjacency);
    
end

%% Helper functions
function qrand = rand_config(xmax, ymax)
    arguments
        xmax double;
        ymax double;
    end
    
    qrand = [xmax; ymax] .* rand(2,1);

end

function [qnear, min_idx, min_dist] = nearest_config(G, q)
    arguments
        G graph;
        q (2,1);
    end
    
    dists = vecnorm(G.Nodes.q - q', 2, 2);
    
    [min_dist, min_idx] = min(dists);
    
    qnear = G.Nodes.q(min_idx, :)';
    
end

function qnew = step_config(qnear, qrand, dq, xmax, ymax)
    arguments
        qnear (2,1);
        qrand (2,1);
        dq double;
        xmax double;
        ymax double;
    end
   
    qnew = qnear + dq * (qrand - qnear)/norm(qrand - qnear);
    qnew(qnew < 0) = 0;
    
    qnew = min(qnew, [xmax; ymax]);
    
    
end

function colliding = collision_check_point(q, obstacles)
    arguments
        q (2,1);
        obstacles cell;
    end
    colliding = collision_check_line([q, q], obstacles);
end
        

function colliding = collision_check_line(q_line, obstacles)
    arguments
        q_line (2,2);
        obstacles cell;
    end
    
    
    checks = boolean(zeros(length(obstacles), 1));

    for i = 1:length(obstacles)
        checks(i) = isintersect_linepolygon(obstacles{i}, q_line);
    end

    colliding = any(checks);

end