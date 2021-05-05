%% collision_detection.m
%
% function for collision detection of linesegment robot in 2D
%
% - written by: Dimitri Lezcano

function colliding = collision_detection(q, S)
    %% Arguments block
    arguments
        q (:,1); % joint angles
        S struct;
    end
    
    %% Initialization
    colliding = false;
    % check if there are any obstacles
    if ~isfield(S, 'obs')
        return;
    end
    
    %% Check each line segment for all of the obstacles
    robot_linesegs = create_robot(q, S.link_len);

    for i = 1:numel(S.obs) % iterate over all obstacles
        obs_i = S.obs{i};
        for j = 1:size(robot_linesegs,3) % iterate over all edges
            edge_j = robot_linesegs(:,:,j);
            
            % check if robot line segment intersects one of the obstacles
            if isintersect_linepolygon(obs_i, edge_j)
                colliding = true;
                return;
            end
        end
    end
end   
        
        