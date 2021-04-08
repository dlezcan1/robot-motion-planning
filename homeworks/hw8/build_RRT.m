%% build_RRT.m
%
% function to build RRT tree. NEED TO ADD COLLISION CHECKING
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   – ∆q: step size (constant).
%   - system: struct of system parameters, must have
%       - dims (1,2) length and width of the box
%       - epsilon (float) distance threshold to say qnear == qG
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built RRT graph

function [path, V, E, G] = build_RRT(qI, qG, NumNodes, dq, system)
    %% Arguments Block
    arguments
        qI (2,1);
        qG (2,1);
        NumNodes double {mustBeInteger};
        dq double;
        system struct;
    end
    %% Check system arguments
    % dims check
    if ~isfield(system, 'dims')
        error("'system' must have 'dims' field.");
        
    elseif ~isequal(size(system.dims), [1, 2])
        error("'system.dims' must be of size [1,2]");
        
    end
    
    % epsilon check
    if ~isfield(system, 'epsilon')
        system.epsilon = dq/2;
    elseif system.epsilon <= 0
        error("'system.epsilon' must be > 0");
    end
    
    %% Initialize the Graph
    G = addnode(graph, 1);
    G.Nodes.q = qI';
        
    %% Build the graph
    for k = 1:NumNodes
        % generate random configuration
        qrand = rand_config(system);
        
        % find the nearest configuration for dq
        [qnear, qnear_idx] = nearest_config(G, qrand);
        
        % find qnew using constant step-size
        qnew = step_config(qnear, qrand, dq, system);
        
        % add qnew to the graph
        if all(G.Nodes.q == qnew')
            continue;
        end
        G = G.addnode(table(qnew', 'VariableNames', {'q'}));
        G = G.addedge(qnear_idx, size(G.Nodes, 1));
        
        % check to see if we are close enough to the edge
        if norm(qnear - qG) < system.epsilon
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
        path = shortestpath(G, 1,qG_idx(1));
    end
    
    %% Return arguments
    V = G.Nodes.q';
    E = table2array(G.Edges);
    
end

%% Helper functions
function qrand = rand_config(system)
    arguments
        system struct;
    end
    
    qrand = system.dims' .* rand(2,1);

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

function qnew = step_config(qnear, qrand, dq, system)
    arguments
        qnear (2,1);
        qrand (2,1);
        dq double;
        system struct;
    end
   
    qnew = qnear + dq * (qrand - qnear)/norm(qrand - qnear);
    qnew(qnew < 0) = 0;
    
    qnew = min(qnew, system.dims');
    
    
end

    