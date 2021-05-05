%% build_RRT.m
%
% function to build RRT tree. 
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   – ∆q: step size (constant).
%   - random_state_gen: random state generator function
%   - S: struct containing obstacles and robot parameters
%   - q_bound_fn: function for bounding generated q's
%   - dist_metric: callable function for distance metric (default is
%       'euclidean')
%   - epsilon: double consisting of when to add qG to the graph 
%                 (|qnew - qG| < epsilon, default = ∆q)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built RRT graph

function [path, V, E, G] = build_RRT(qI, qG, NumNodes, dq, random_state_gen, ...
                                     S, q_bound_fn, dist_metric, epsilon)
    %% Arguments Block
    arguments
        qI (:,1);
        qG (:,1);
        NumNodes double {mustBeInteger};
        dq double;
        random_state_gen;
        S struct;
        q_bound_fn;
        dist_metric = @(X,x) vecnorm(X - x, 2, 2);
        epsilon double = dq;
    end
    
    %% Initialize the Graph
    G = addnode(graph, 1);
    G.Nodes.q = qI';
        
    %% Build the graph
    for k = 1:NumNodes
        % generate random configuration
        qrand = random_state_gen();
        
        % find the nearest configuration for dq
        [qnear, qnear_idx] = nearest_config(G, qrand, dist_metric);
        
        % find qnew using constant step-size
        qnew = step_config(qnear, qrand, dq, q_bound_fn);
        
        % Check if point is already in the graph
        if all(G.Nodes.q == qnew')
            continue;
        end
        
        % collision checking
        colliding = collision_check_line(qnear, qnew, S, dq);
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
function [qnear, min_idx, min_dist] = nearest_config(G, q, dist_metric)
    arguments
        G graph;
        q (:,1);
        dist_metric;
    end
    
    dists = dist_metric(G.Nodes.q', q);
    
    [min_dist, min_idx] = min(dists);
    
    qnear = G.Nodes.q(min_idx, :)';
    
end

function qnew = step_config(qnear, qrand, dq, q_bound_fn)
    arguments
        qnear (:,1);
        qrand (:,1);
        dq double;
        q_bound_fn;
    end
   
    qnew = qnear + dq * (qrand - qnear)/norm(qrand - qnear);
    
    qnew = q_bound_fn(qnew);
    
end

% sample points along a line (in C-Space) to check for a collision
function colliding = collision_check_line(q1, q2, S, dq)
    arguments
        q1 (:,1);
        q2 (:,1);
        S struct;
        dq double = 0.1;
    end
    
    % generate the sample q's along this line
    q_checks = q1 + linspace(0, 1, dq).*(q2 - q1);
    
    % initialization
    colliding = false;
    
    % iterate through all of the sample q's
    for i = 1:size(q_checks, 2)
        % check for collision
        if collision_detection(q_checks(:,i), S)
            colliding = true;
            return;
        end
    end
end 