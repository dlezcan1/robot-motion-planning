%% build_RRT.m
%
% function to build RRT tree. 
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   – ∆q: step size (constant).
%   - obstacles: cell array of 2xN matrices (default is {})
%   - xmax, ymax: Positive doubles representing C-space bounds (default = 1,
%                   xmax)
%   - epsilon: double consisting of when to add qG to the graph 
%                 (|qnew - qG| < epsilon, default = ∆q)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built RRT graph

function [path, V, E, G] = build_RRT(qI, qG, NumNodes, dq, obstacles, xmax, ymax,...
                                     epsilon)
    %% Arguments Block
    arguments
        qI (2,1);
        qG (2,1);
        NumNodes double {mustBeInteger};
        dq double;
        obstacles cell = {};
        xmax double {mustBePositive} = 1;
        ymax double {mustBePositive} = xmax;
        epsilon double = dq;
    end
    %% Initialize the Graph
    G = addnode(graph, 1);
    G.Nodes.q = qI';
        
    %% Build the graph
    for k = 1:NumNodes
        % generate random configuration
        qrand = rand_config(xmax, ymax);
        
        % find the nearest configuration for dq
        [qnear, qnear_idx] = nearest_config(G, qrand);
        
        % find qnew using constant step-size
        qnew = step_config(qnear, qrand, dq, xmax, ymax);
        
        % Check if point is already in the graph
        if all(G.Nodes.q == qnew')
            continue;
        end
        
        % collision checking
        colliding = collision_check([qnear, qnew], obstacles);
        if colliding
            continue;
        end
        
        G = G.addnode(table(qnew', 'VariableNames', {'q'}));
        G = G.addedge(qnear_idx, size(G.Nodes, 1));
        
        % check to see if we are close enough to the edge
        if norm(qnear - qG) < epsilon
            G = G.addnode(table(qG', 'VariableNames', {'q'}));
            G = G.addedge(qnear_idx, size(G.Nodes, 1));
            break;
        end
    end
    
    %% Determine the path
    qG_idx = find(all(G.Nodes.q == qG', 2));
    if isempty(qG_idx)
        path = [];
    else
        idx_path = shortestpath(G, 1,qG_idx(1));
        path = G.Nodes.q(idx_path, :)';
    end
    
    %% Return arguments
    V = G.Nodes.q';
    E = table2array(G.Edges);
    
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

function colliding = collision_check(q_line, obstacles)
    arguments
        q_line (2,2);
        obstacles {cell};
    end
    
    
    checks = boolean(zeros(length(obstacles), 1));

    for i = 1:length(obstacles)
        checks(i) = isintersect_linepolygon(obstacles{i}, q_line);
    end

    colliding = any(checks);

end

    