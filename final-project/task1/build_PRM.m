%% build_PRM.m
%
% function to build PRM.
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   - K: the number of nearest neighbors
%   - random_state_gen: function handle to generate random state
%   - S: struct containing robot and obstacle parameters
%   - dist_metric: function handle for distance metric (default is 'euclidean')
%   - obs_detect_dq: float for sampling obstacle points (default is 0.1)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built PRM


function [path, V, E, G] = build_PRM(qI, qG, NumNodes, K, random_state_gen, ...
                                      S, dist_metric, options)
    %% Arguments Block
    arguments
        qI (:,1);
        qG (:,1);
        NumNodes double {mustBeInteger};
        K double {mustBeInteger};
        random_state_gen;
        S struct;
        dist_metric = 'euclidean';
        options.obstacle_dq double = 0.01;
        options.line_wraparound logical = true;
    end
    
    % Argument checking
    if length(qI) ~= length(qG)
        error("qI and qG are not the same size");
    end
    
    %% Initialization
    G = addnode(graph, 2);
    G.Nodes.q = [qI, qG]';
    
    %% Generate number of nodes
    while G.numnodes < NumNodes
       % generate qrand
       qrand = random_state_gen();
       
       % check for collision
       colliding = collision_detection(qrand, S);
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
        knn_idx = knnsearch(G.Nodes.q([1:j-1, j+1:end],:), q_j', 'K', K, 'Distance', dist_metric);
        
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
           colliding = collision_check_line(q_j, q_idx, S, options.obstacle_dq,...
                            'wraparound', options.line_wraparound);
                      
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