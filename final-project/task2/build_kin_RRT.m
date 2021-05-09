%% build_RRT.m
%
% function to build RRT tree. 
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   – ∆q: step size (constant).
%   - random_state_gen: random state generator function
%   - kinemtics: function(q,u) that provides kinematic update
%   - controls: cell containing discretized possible controls
%   - obstacles: cell containing all of the obstacles (default is {})
%   - dist_metric: callable function for distance metric (default is
%       'euclidean')
%   - collision_check: function to check for collisions (default is disabled
%       collisiong checking)
%   - epsilon: double consisting of when to add qG to the graph 
%                 (|qnew - qG| < epsilon, default = ∆q)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built RRT graph

function [path, V, E, G] = build_kin_RRT(qI, qG, NumNodes, dt, random_state_gen, ...
                                         kinematics, controls, epsilon, ...
                                         obstacles, collision_check, dist_metric)
    %% Arguments Block
    arguments
        qI (:,1);
        qG (:,1);
        NumNodes double {mustBeInteger};
        dt double;
        random_state_gen;
        kinematics;
        controls cell;
        epsilon double;
        obstacles cell = {};
        collision_check = @(q, obs) false; % assume false
        dist_metric = @(x,X) vecnorm(X - x, 2, 1);
    end
    
    %% Initialize the Graph
    G = addnode_state(graph, qI);
    G.Edges.controls = zeros(0,numel(controls{1})); % initialize controls
        
    %% Build the graph
    for k = 1:NumNodes
        % generate random configuration
        qrand = random_state_gen();
        
        % find the nearest configuration for qrand
        [qnear, qnear_idx] = nearest_config(G, qrand, dist_metric);
        
        % find qnew using constant time-step
        [qnew,u] = step_config(qnear, qrand, dt, kinematics, controls, dist_metric);
        
        % Check if point is already in the graph
        if any(all(G.Nodes.q' == qnew))
            continue;
        end
        
        % collision checking
        colliding = collision_check(qnew, obstacles);
        if colliding
            continue;
        end
        
        G = addnode_state(G, qnew);
        G = addedge_control(G, qnear_idx, size(G.Nodes,1), u);
        
        % check to see if we are close enough to the edge
        if dist_metric(qnear, qG) < epsilon
            G = addnode_state(G, qG);
            qnew_idx = find(all(G.Nodes.q == qnew',2));
            G = addedge_control(G, qnew_idx(1), size(G.Nodes, 1), zeros(size(controls{1})));
            break;
        end
    end
    
    %% Determine the path
    qG_idx = find(all(G.Nodes.q == qG', 2));
    if isempty(qG_idx)
        path = [];
    else
        idx_path = shortestpath(G, 1, qG_idx(1));
        path = G.Nodes.q(idx_path, :)';
    end
    
    %% Return arguments
    V = G.Nodes.q';
    E = full(G.adjacency);
    
end

%% Helper functions
function [qnear, min_idx, min_dist] = nearest_config(G, q, dist_metric)
    arguments
        G graph;
        q (:,1);
        dist_metric;
    end
    
    dists = dist_metric(q, G.Nodes.q');
    
    [min_dist, min_idx] = min(dists);
    
    qnear = G.Nodes.q(min_idx, :)';
    
end

function [qnew,u] = step_config(qnear, qrand, dt, kinematics, controls, dist_fn)
    arguments
        qnear (:,1);
        qrand (:,1);
        dt double;
        kinematics;
        controls cell;
        dist_fn;
    end
   
    % determine all of the possible q's from control
    min_dist = inf;
    for i = 1:numel(controls)
        % update kinematic state
        u_i = controls{i};
        q_i = kinematics(qnear, u_i, dt);
        
        % compute distance to qrand
        dist_i = dist_fn(q_i, qrand);
        
        % update if closer
        if dist_i < min_dist
            min_dist = dist_i;
            qnew = q_i;
            u = u_i;
        end 
    end
end

% graph structure to add parameters to graphs
function Gnew = addedge_control(G, i, j, u_ij)
    arguments
        G graph;
        i {mustBeInteger};
        j {mustBeInteger};
        u_ij (:,1);
    end
    
    newedge = table([i, j], u_ij', 'VariableNames', G.Edges.Properties.VariableNames);
    
    Gnew = G.addedge(newedge);
    
end

function Gnew = addnode_state(G, q)
    arguments
        G graph;
        q (:,1);
    end
    
    Gnew = G.addnode(table(q', 'VariableNames', {'q'}));
end

        